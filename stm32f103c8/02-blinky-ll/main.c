/*
 * Bare metal blinky for STM32F103C8
 */

#include <stdint.h>
#include <stm32f1xx.h>
#include <stm32f1xx_ll_bus.h>
#include <stm32f1xx_ll_gpio.h>

int main(void) {
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_13, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(GPIOC, LL_GPIO_PIN_13, LL_GPIO_OUTPUT_PUSHPULL);
    while(1) {
        LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);
        for (int i = 0; i < 100000; i++); // arbitrary delay
    }
    return 0;
}
