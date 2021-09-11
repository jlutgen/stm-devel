/*
 * Bare metal blinky for Nucleo-F091
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

int main(void) {  
    // Enable peripheral bus clock for Port A
    RCC_AHBENR |= RCC_AHBENR_GPIOAEN;

    // PA5: general-purpose output (push-pull is default)
    GPIOA_MODER |= GPIO_MODE(5, GPIO_MODE_OUTPUT);

    while (1) {
        GPIOA_ODR ^= 1 << 5;  // toggle user LED on PA5
        for (int i = 0; i < 200000; i++)  // arbitrary delay
            ;
    }
    return 0;
}
