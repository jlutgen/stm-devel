/*
 * Bare metal blinky for Nucleo-F091
 */

#include <stdint.h>
#include <stm32f0xx.h>

int main(void) {  
    // Enable peripheral bus clock for Port A
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    // PA5: general-purpose output (push-pull is default)
    GPIOA->MODER |= 1 << (5 * 2);

    while (1) {
        GPIOA->ODR ^= 1 << 5;  // toggle user LED on PA5
        for (int i = 0; i < 200000; i++)  // arbitrary delay
            ;
    }
    return 0;
}
