/*
 * Test Timer 14 on Nucleo-F091 using LL library
 *
 * Toggle a pin in the ISR for TIM14 interrupt
 */

#include <stdint.h>
#include <stm32f0xx.h>
#include <stm32f0xx_ll_bus.h>
#include <stm32f0xx_ll_gpio.h>
#include <stm32f0xx_ll_rcc.h>
#include <stm32f0xx_ll_system.h>
#include <stm32f0xx_ll_tim.h>
#include <stm32f0xx_ll_utils.h>

void config_clocks(void) {
    // Configure system clock, AHB clock, and APB clock for 48 MHz operation
    LL_UTILS_ClkInitTypeDef clkinit;
    LL_UTILS_PLLInitTypeDef pllinit;
    clkinit.AHBCLKDivider = LL_RCC_SYSCLK_DIV_1;
    clkinit.APB1CLKDivider = LL_RCC_APB1_DIV_1;
    pllinit.PLLMul = LL_RCC_PLL_MUL_6;
    pllinit.PLLDiv = LL_RCC_PREDIV_DIV_1;
    LL_PLL_ConfigSystemClock_HSI(&pllinit, &clkinit);
}

void config_1ms_tick(void) {
    LL_RCC_ClocksTypeDef clocks;
    LL_RCC_GetSystemClocksFreq(&clocks);
    LL_Init1msTick(clocks.HCLK_Frequency);
}

void config_gpio(void) {
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

    // User LED on PA5
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_5, LL_GPIO_OUTPUT_PUSHPULL);

    // Timer ISR toggles PA3
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_10, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_10, LL_GPIO_OUTPUT_PUSHPULL);
}

void config_timer14() {
    // TIM14 is general-purpose auto-reload 16-bit upcounter
    // Counter is clocked by APB clock by default (freq = PCLK)
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM14);
    LL_TIM_SetAutoReload(TIM14, 0x7fff);
    LL_TIM_SetPrescaler(TIM14, 0); // overflow freq = PCLK / (reload + 1)
    LL_TIM_EnableIT_UPDATE(TIM14);
}

// ISR names are in startup_stm32*.s
void TIM14_IRQHandler(void) {
    LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_10);
    LL_TIM_ClearFlag_UPDATE(TIM14);
}

int main(void) {
    config_clocks();
    config_1ms_tick();
    config_gpio();
    config_timer14();
    NVIC_EnableIRQ(TIM14_IRQn);
    LL_TIM_EnableCounter(TIM14);
    while (1) {
        LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_5);
        LL_mDelay(1000);
        LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_5);
        LL_mDelay(1000);
    }
    return 0;
}
