/*
 * Try the user button connected to PC13 on the Nucleo-F091RC
 */

#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

void config_clocks(void) {
    // Configure system clock for 48 MHz operation

    // HSION = 1 by default (internal 8 MHz RC oscillator)
    // and HSI is selected as system clock by default.
    // PLL is off and not ready (i.e., is unlocked) by default.

    // Set the PLL source and  multiplier
    // Constants aren't defined in an entirely consistent way in libopencm3 header
    // files; for example, we must shift RCC_CFGR_PLLSRS_HSI_CLK_DIV2 to the correct
    // position ourselves, but RCC_CFGR_PLLMUL_MUL12 incorporates the correct shift
    // already.
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
}

void config_gpio(void) {
    RCC_AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC_AHBENR |= RCC_AHBENR_GPIOCEN;

    // User LED on PA5
    // Config as general-purpose output (push-pull low-speed is default)
    GPIOA_MODER |= GPIO_MODE(5, GPIO_MODE_OUTPUT);

    // User button on PC13
    // Config as floating low-speed digital input (default)
}

int main(void) {
    config_clocks();
    config_gpio();
    while (1) {
        if (GPIOC_IDR & (1 << 13))  // button not pressed
            GPIOA_ODR &= ~(1 << 5);
        else
            GPIOA_ODR |= 1 << 5;
    }
    return 0;
}
