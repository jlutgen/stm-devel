/*
 * Try the user button connected to PC13 on the Nucleo-F091RC
 */

#include <stm32f0xx.h>

#define GPIO_MODE_OUTPUT 1
#define HCLK 48000000

void config_clocks(void) {
    // Configure system clock, AHB clock, and APB clock for 48 MHz operation

    // Set the PLL source and  multiplier (PREDIV is divide by 1 by default)
    RCC->CFGR |= RCC_CFGR_PLLSRC_HSI_PREDIV | RCC_CFGR_PLLMUL6;
    // Enable the PLL
    RCC->CR |= RCC_CR_PLLON;
    // Wait for PLL ready
    while (!(RCC->CR & RCC_CR_PLLRDY))
        ;

    // One wait state needed if sysclk > 24 MHz
    FLASH->ACR |= FLASH_ACR_LATENCY;

    // Select PLL as system clock
    RCC->CFGR |= RCC_CFGR_SW_PLL;

    // Wait until PLL is switched on
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
        ;
}

void config_1ms_tick(void) {
    // Configure SysTick for 1 ms interrupts

    SysTick->LOAD = HCLK / 1000 - 1;  // reload value
    SysTick->VAL = 0;                 // current value
    // Use processor clock, and enable
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
}

void delay_ms(int ms) {
    SysTick->VAL = 0;  // reset to zero; also, writing to VAL clears COUNTFLAG
    while (ms) {
        if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
            // SysTick timer has counted down to zero, so 1 ms has elapsed
            ms--;
        }
    }
}

void config_gpio(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

    // User LED on PA5
    // Config as general-purpose output (push-pull low-speed is default)
    GPIOA->MODER |= (GPIO_MODE_OUTPUT << GPIO_MODER_MODER5_Pos);

    // User button on PC13
    // Config as floating low-speed digital input (default)
}

int main(void) {
    config_clocks();
    config_1ms_tick();
    config_gpio();
    while (1) {
        if (GPIOC->IDR & (1 << 13))  // button not pressed
            GPIOA->ODR &= ~(1 << 5);
        else
            GPIOA->ODR |= 1 << 5;
    }
    return 0;
}
