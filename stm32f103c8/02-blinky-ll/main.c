/*
 * Blinky for STM32F103C8 using LL
 */

#include <stdint.h>
#include <stm32f1xx.h>
#include <stm32f1xx_ll_bus.h>
#include <stm32f1xx_ll_gpio.h>
#include <stm32f1xx_ll_rcc.h>
#include <stm32f1xx_ll_system.h>
#include <stm32f1xx_ll_utils.h>

void config_1ms_tick(void) {
    LL_RCC_ClocksTypeDef clocks;
    LL_RCC_GetSystemClocksFreq(&clocks);
    LL_Init1msTick(clocks.HCLK_Frequency);
}

void config_gpio(void) {
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_13, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(GPIOC, LL_GPIO_PIN_13, LL_GPIO_OUTPUT_PUSHPULL);
}

int main(void) {
    config_1ms_tick();
    config_gpio();

    while(1) {
        LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);
        LL_mDelay(500);
    }
    return 0;
}
