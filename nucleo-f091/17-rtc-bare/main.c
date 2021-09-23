/*
 * RTC
 *
 */

#include <stdio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/systick.h>

#define HCLK 48000000

static char msg[80];

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
    RCC_APB1ENR |= RCC_APB1ENR_PWREN; // Enable the PWR module
    PWR_CR |= PWR_CR_DBP; // Allow access to RCC_BDCR bits
    RCC_BDCR |= RCC_BDCR_LSEON; // Turn on the LSE
    RCC_BDCR |= RCC_BDCR_RTCSEL_LSE; // RCC clock is LSE
    RCC_BDCR |= RCC_BDCR_RTCEN; // Enable RCC clock
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

    GPIOA_MODER |= GPIO_MODE(4, GPIO_MODE_AF);  // AF mode
    GPIOA_AFRL |= GPIO_AFR(4, GPIO_AF4);        // AF #4 on PA4 is TIM14_CH1

    // PA0: analog input (ADC_CH0)
    GPIOA_MODER |= GPIO_MODE(0, GPIO_MODE_ANALOG);
}

void config_uart(void) {
    RCC_APB1ENR |= RCC_APB1ENR_USART2EN;  // Enable USART2 on APB

    // Defaults: 8 data bits, 1 stop bit, no parity (8N1)

    // Set baud rate to 115200
    USART2_BRR = 48000000 / 115200;
    // tx enable, rx enable, uart enable
    USART2_CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

void config_adc(void) {
    RCC_APB2ENR |= RCC_APB2ENR_ADCEN;  // Enable the peripheral clock for ADC
    ADC1_CR |= ADC_CR_ADEN;            // Enable the ADC
    // Wait until the ADC is ready
    while (!(ADC1_ISR & ADC_ISR_ADRDY))
        ;
    ADC1_CFGR2 |= ADC_CFGR2_CKMODE_PCLK_DIV2;  // ADC clock is PCLK / 2
    ADC1_CHSELR |= ADC_CHSELR_CHSEL(16);  // Select ch 16 (temperature sensor)
    ADC1_CHSELR |= ADC_CHSELR_CHSEL(0);   // ... and ch 0 (PA0/ADC_IN0)
    ADC1_CCR |= ADC_CCR_TSEN;             // Enable temperature sensor
    // Datasheet says min sampling time for temperature sensor
    // is 4 us, which is 80 ADC clock cycles (ADC clock: 20 MHz)
    ADC1_SMPR |= ADC_SMPR_SMP_239DOT5;  // Sampling time: 239.5 cycles
}

void set_rtc_clock(void) {
    // Unlock RTC registers
    RTC_WPR = 0xCA;
    RTC_WPR = 0x53;

    RTC_ISR |= RTC_ISR_INIT; // Enter initialization mode
    while (!(RTC_ISR & RTC_ISR_INITF))
        ; // Wait until init flag set
    // TODO
    
}

int main(void) {
    int32_t temperature;

    config_clocks();
    config_1ms_tick();
    config_gpio();
    config_uart();
    config_adc();
    if (RCC_BDCR & RCC_BDCR_LSERDY)
        usart_write(USART2, "LSE ready\r\n");
    else
        usart_write(USART2, "LSE not ready\r\n");
    while (1) {
        // Start a sequence of ADC conversions, one for
        // each channel we have enabled
        ADC1_CR |= ADC_CR_ADSTART;

        // Channel 0
        while (!(ADC1_ISR & ADC_ISR_EOC))
            ;  // Wait for conversion to be completed
        sprintf(msg, "%ld\t", ADC1_DR);
        usart_write(USART2, msg);

        // Channel 16
        while (!(ADC1_ISR & ADC_ISR_EOC))
            ;  // Wait for conversion to be completed
        temperature = ADC1_DR;
        sprintf(msg, "%ld\r\n", temperature);
        usart_write(USART2, msg);
        delay_ms(2000);
    }
    return 0;
}
