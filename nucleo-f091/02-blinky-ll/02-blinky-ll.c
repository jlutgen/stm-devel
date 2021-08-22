/*
 * Blinky for Nucleo-F091 using LL library
 */

#include <stdint.h>
#include <stm32f0xx.h>
#include <stm32f0xx_ll_bus.h>
#include <stm32f0xx_ll_gpio.h>
#include <stm32f0xx_ll_rcc.h>

int main(void) {
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_8);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_5, LL_GPIO_OUTPUT_PUSHPULL);
    while(1) {
        LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5);
        for (int i = 0; i < 200000; i++); // arbitrary delay
        LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5);
        for (int i = 0; i < 400000; i++); // twice the arbitrary delay
    }
    return 0;
}
