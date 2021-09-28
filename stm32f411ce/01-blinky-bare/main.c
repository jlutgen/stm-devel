/*
 * Bare metal blinky for Nucleo-F091
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

int main(void) {  
    // Enable peripheral bus clock for Port C
    RCC_AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    // PC13: general-purpose output (push-pull is default)
    GPIOC_MODER |= GPIO_MODE(13, GPIO_MODE_OUTPUT);

    while (1) {
        GPIOC_ODR ^= 1 << 13;  // toggle user LED on PA5
        for (int i = 0; i < 200000; i++)  // arbitrary delay
            ;
    }
    return 0;
}
