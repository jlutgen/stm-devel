/*
 * DAC
 */

#include <libopencm3/stm32/dac.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/systick.h>

#define HCLK 48000000

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

    GPIOA_MODER |= GPIO_MODE(8, GPIO_MODE_AF);  // AF mode
    GPIOA_AFRH |= GPIO_AFR(8 - 8, GPIO_AF0);    // AF #0 on PA8 is MCO
    GPIOA_OSPEEDR |= GPIO_OSPEED(8, GPIO_OSPEED_100MHZ);

    // PA4: DAC channel 1 output: analog
    GPIOA_MODER |= GPIO_MODE(4, GPIO_MODE_ANALOG);
}

void config_dac(void) {

    DAC_CR(DAC1) |= DAC_CR_EN1; // Enable DAC channel 1
}

int main(void) {
    config_clocks();
    config_1ms_tick();
    config_gpio();
    config_dac();
    while (1) {

        delay_ms(1000);
    }
    return 0;
}
