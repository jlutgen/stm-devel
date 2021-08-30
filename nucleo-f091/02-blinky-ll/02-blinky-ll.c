/*
 * Blinky for Nucleo-F091 using LL library
 */

#include <stdint.h>
#include <stm32f0xx.h>
#include <stm32f0xx_ll_bus.h>
#include <stm32f0xx_ll_gpio.h>

int main(void) {
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

    // User LED on PA5: output (push-pull by default)
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);

    while (1) {
        LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_5);
        for (int i = 0; i < 200000; i++) // arbitrary delay
            ;
    }
    return 0;
}
