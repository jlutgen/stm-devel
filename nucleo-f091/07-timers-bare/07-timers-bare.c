/*
 * Test Timer 14 on Nucleo-F091 using LL library
 *
 * Toggle a pin in the ISR for TIM14 interrupt
 */

#include <stdint.h>
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

    FLASH->ACR |= FLASH_ACR_LATENCY;  // one wait state needed if sysclk will be
                                      // over 24 MHz
    // Select PLL as system clock
    RCC->CFGR |= RCC_CFGR_SW_PLL;

    // Wait until PLL is switched on
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
        ;
}

void config_1ms_tick(void) {
    // Configure SysTick for 1 ms interrupts

    SysTick->LOAD = HCLK / 1000 - 1; // reload value
    SysTick->VAL = 0;                // current value
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

    // User LED on PA5
    // Set PA5 to general purpose output mode (push-pull low-speed is default)
    GPIOA->MODER |= (GPIO_MODE_OUTPUT << GPIO_MODER_MODER5_Pos);

    // Timer ISR toggles PA10
    GPIOA->MODER |= (GPIO_MODE_OUTPUT << GPIO_MODER_MODER10_Pos);
}

void config_timer14() {
    // TIM14 is general-purpose auto-reload 16-bit upcounter
    // Counter is clocked by APB clock by default (freq = PCLK)
    // Clock tick freq = PCLK / (PSC + 1)
    // Overflow freq = (clock tick freq) / (reload + 1)
    RCC->APB1ENR |= RCC_APB1ENR_TIM14EN; // enable clock for TIM14
    TIM14->ARR = 0x7fff; // reload value
    TIM14->PSC = 0; // divide input clock by 1
    TIM14->DIER |= TIM_DIER_UIE; // enable interrupts for update event (timer overflow)
}

// ISR names are in startup_stm32*.s
void TIM14_IRQHandler(void) {
    GPIOA->ODR ^= (1 << 10);   // toggle PA10
    TIM14->SR &= ~TIM_SR_UIF;  // clear the update interrupt flag
}

int main(void) {
    config_clocks();
    config_1ms_tick();
    config_gpio();
    config_timer14();
    NVIC_EnableIRQ(TIM14_IRQn);
    TIM14->CR1 |= TIM_CR1_CEN; // start TIM14
    while (1) {
        // Toggle user LED on PA5
        GPIOA->ODR ^= (1 << 5);
        delay_ms(1000);
        GPIOA->ODR ^= (1 << 5);
        delay_ms(1000);
    }
    return 0;
}
