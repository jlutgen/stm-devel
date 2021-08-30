/*
 * Try the user button connected to PC13 on the Nucleo-F091RC
 */

#include <stm32f0xx.h>
#include <stm32f0xx_ll_bus.h>   // LL_AHB1_GRP1_EnableClock
#include <stm32f0xx_ll_gpio.h>
#include <stm32f0xx_ll_rcc.h>
#include <stm32f0xx_ll_utils.h> // LL_PLL_ConfigSystemClock_HSI

#define HCLK 48000000

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

void config_gpio(void) {
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);

    // User LED on PA5: output (push-pull by default)
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);

    // User button on PC13: floating low-speed digital input (default)
}

int main(void) {
    config_clocks();
    config_gpio();
    while (1) {
        if (LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_13)) // button not pressed
            LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5);
        else
            LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5);
    }
    return 0;
}
