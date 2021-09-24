/*
 * RTC
 */

#include <stdio.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/systick.h>

#define HCLK 48000000

static char msg[80];
static char* day_names[] = {
    "Monday", "Tuesday",  "Wednesday", "Thursday",
    "Friday", "Saturday", "Sunday",
};

struct datetime {
    int8_t yy;
    int8_t mo;
    int8_t dd;
    int8_t day_of_week;
    int8_t hh;
    int8_t mm;
    int8_t ss;
};

void usart_read(uint32_t usart, char* s, int len) {
    for (int i = 0; i < len; i++) {
        while ((USART_ISR(usart) & USART_ISR_RXNE) != USART_ISR_RXNE)
            ;
        *s = (uint8_t)(USART_RDR(usart)); /* Receive data, clear flag */
        if (*s == '\r' || *s == '\n') {
            *(s + 1) = '\0';
            return;
        }
        ++s;
    }
}

void usart_write(uint32_t usart, char* s) {
    while (*s) {
        USART_TDR(usart) = *s++;
        // Wait for data to be shifted from TDR to tx shift register
        while (!(USART_ISR(usart) & USART_ISR_TXE))
            ;
    }
    // Wait for last data to be transmitted (transmission complete)
    while (!(USART_ISR(usart) & USART_ISR_TC))
        ;
}

void config_clocks(void) {
    // Configure system clock for 48 MHz operation

    // HSION = 1 by default (internal 8 MHz RC oscillator)
    // and HSI is selected as system clock by default.
    // PLL is off and not ready (i.e., is unlocked) by default.

    // Set the PLL source and  multiplier
    // Constants aren't defined in an entirely consistent way in libopencm3
    // header files; for example, we must shift RCC_CFGR_PLLSRS_HSI_CLK_DIV2 to
    // the correct position ourselves, but RCC_CFGR_PLLMUL_MUL12 incorporates
    // the correct shift already.
    RCC_CFGR |= (RCC_CFGR_PLLSRC_HSI_CLK_DIV2 << 16) | RCC_CFGR_PLLMUL_MUL12;
    // Enable the PLL
    RCC_CR |= RCC_CR_PLLON;
    // Wait for PLL ready
    while (!(RCC_CR & RCC_CR_PLLRDY))
        ;

    // One wait state needed if SYSCLK > 24 MHz
    FLASH_ACR |= FLASH_ACR_LATENCY_1WS;

    // Select PLL as system clock
    RCC_CFGR |= RCC_CFGR_SW_PLL;

    // Wait until PLL is switched on
    while ((RCC_CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
        ;

    // Make system clock (divided by prediv) available on MCO signal
    // RCC_CFGR |= RCC_CFGR_MCOPRE_DIV128;
    // RCC_CFGR |= RCC_CFGR_MCO_SYSCLK << RCC_CFGR_MCO_SHIFT;

    // Make LSE available on MCO signal
    RCC_CFGR |= RCC_CFGR_MCO_LSE << RCC_CFGR_MCO_SHIFT;

    // Start the 32.768 kHz low-speed external oscillator (LSE)
    RCC_APB1ENR |= RCC_APB1ENR_PWREN;  // Enable the PWR module
    PWR_CR |= PWR_CR_DBP;              // Allow access to RCC_BDCR bits
    RCC_BDCR |= RCC_BDCR_LSEON;        // Turn on the LSE
    while (!(RCC_BDCR & RCC_BDCR_LSERDY))
        ;                             // Wait for LSE ready
    RCC_BDCR |= RCC_BDCR_RTCSEL_LSE;  // RCC clock is LSE
    RCC_BDCR |= RCC_BDCR_RTCEN;       // Enable RCC clock
}

void config_1ms_tick(void) {
    // Configure SysTick for 1 ms interrupts

    STK_RVR = HCLK / 1000 - 1;  // reload value
    STK_CVR = 0;                // current value
    // Use processor clock, and enable
    STK_CSR = STK_CSR_CLKSOURCE_AHB | STK_CSR_ENABLE;
}

void delay_ms(int ms) {
    STK_CVR = 0;  // reset to zero; also, writing to CVR clears COUNTFLAG
    while (ms) {
        if (STK_CSR & STK_CSR_COUNTFLAG) {
            // SysTick timer has counted down to zero, so 1 ms has elapsed
            ms--;
        }
    }
}

void config_gpio(void) {
    // Must enable a peripheral's clock before writing to its registers
    RCC_AHBENR |= RCC_AHBENR_GPIOAEN;  // Enable Port A clock

    GPIOA_MODER |= GPIO_MODE(2, GPIO_MODE_AF);  // AF mode
    GPIOA_AFRL |= GPIO_AFR(2, GPIO_AF1);        // AF #1 on PA2 is USART2_TX

    GPIOA_MODER |= GPIO_MODE(3, GPIO_MODE_AF);  // AF mode
    GPIOA_AFRL |= GPIO_AFR(3, GPIO_AF1);        // AF #1 on PA3 is USART2_RX

    GPIOA_MODER |= GPIO_MODE(8, GPIO_MODE_AF);  // AF mode
    GPIOA_AFRH |= GPIO_AFR(8 - 8, GPIO_AF0);    // AF #0 on PA8 is MCO
    GPIOA_OSPEEDR |= GPIO_OSPEED(8, GPIO_OSPEED_100MHZ);
}

void config_uart(void) {
    RCC_APB1ENR |= RCC_APB1ENR_USART2EN;  // Enable USART2 on APB

    // Defaults: 8 data bits, 1 stop bit, no parity (8N1)

    // Set baud rate to 115200
    USART2_BRR = 48000000 / 115200;
    // tx enable, rx enable, uart enable
    USART2_CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

void set_rtc_clock(void) {
    // Unlock RTC registers
    RTC_WPR = 0xCA;
    RTC_WPR = 0x53;

    RTC_ISR |= RTC_ISR_INIT;  // Enter initialization mode
    while (!(RTC_ISR & RTC_ISR_INITF))
        ;  // Wait until init flag set
    // Program date: 2021-09-24
    RTC_DR = (2 << RTC_DR_YT_SHIFT) | (1 << RTC_DR_YU_SHIFT) |
             (RTC_DR_WDU_FRI << RTC_DR_WDU_SHIFT) | (9 << RTC_DR_MU_SHIFT) |
             (2 << RTC_DR_DT_SHIFT) | (4 << RTC_DR_DU_SHIFT);
    RTC_ISR &= ~RTC_ISR_INIT;  // Exit initialization mode
}

void read_rtc_clock(struct datetime* dt) {
    uint32_t dr = RTC_DR;
    uint32_t tr = RTC_TR;
    dt->yy =
        10 * ((dr & (RTC_DR_YT_MASK << RTC_DR_YT_SHIFT)) >> RTC_DR_YT_SHIFT);
    dt->yy += ((dr & (RTC_DR_YU_MASK << RTC_DR_YU_SHIFT)) >> RTC_DR_YU_SHIFT);
    dt->day_of_week =
        ((dr & (RTC_DR_WDU_MASK << RTC_DR_WDU_SHIFT)) >> RTC_DR_WDU_SHIFT);
    dt->mo =
        10 * ((dr & (RTC_DR_MT_MASK << RTC_DR_MT_SHIFT)) >> RTC_DR_MT_SHIFT);
    dt->mo += ((dr & (RTC_DR_MU_MASK << RTC_DR_MU_SHIFT)) >> RTC_DR_MU_SHIFT);
    dt->dd =
        10 * ((dr & (RTC_DR_DT_MASK << RTC_DR_DT_SHIFT)) >> RTC_DR_DT_SHIFT);
    dt->dd += ((dr & (RTC_DR_DU_MASK << RTC_DR_DU_SHIFT)) >> RTC_DR_DU_SHIFT);
    dt->hh =
        10 * ((tr & (RTC_TR_HT_MASK << RTC_TR_HT_SHIFT)) >> RTC_TR_HT_SHIFT);
    dt->hh += ((tr & (RTC_TR_HU_MASK << RTC_TR_HU_SHIFT)) >> RTC_TR_HU_SHIFT);
    dt->mm =
        10 * ((tr & (RTC_TR_MNT_MASK << RTC_TR_MNT_SHIFT)) >> RTC_TR_MNT_SHIFT);
    dt->mm +=
        ((tr & (RTC_TR_MNU_MASK << RTC_TR_MNU_SHIFT)) >> RTC_TR_MNU_SHIFT);
    dt->ss =
        10 * ((tr & (RTC_TR_ST_MASK << RTC_TR_ST_SHIFT)) >> RTC_TR_ST_SHIFT);
    dt->ss += ((tr & (RTC_TR_SU_MASK << RTC_TR_SU_SHIFT)) >> RTC_TR_SU_SHIFT);
}

int main(void) {
    struct datetime dt;

    config_clocks();
    config_1ms_tick();
    config_gpio();
    config_uart();
    set_rtc_clock();
    while (1) {
        read_rtc_clock(&dt);
        sprintf(msg, "%s 20%02d-%02d-%02d %02d:%02d:%02d\r\n",
                day_names[dt.day_of_week - 1], dt.yy, dt.mo, dt.dd, dt.hh,
                dt.mm, dt.ss);
        usart_write(USART2, msg);

        delay_ms(1000);
    }
    return 0;
}
