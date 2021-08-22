/*
 * Bare metal blinky for Nucleo-F091
 */

#include <stdint.h>
#include <stm32f0xx.h>

#define BIT_5 (1UL << 5)

int main(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;  // enable peripheral bus clock for Port A

    // GPIOA->MODER &= 0xFFFFF3FF;  // clear mode bits for PA5
    GPIOA->MODER &= ~GPIO_MODER_MODER5_Msk;  // clear mode bits for PA5

    // GPIOA->MODER |= 0x00000400;  // PA5: general-purpose push-pull output, max speed 2 MHz
    GPIOA->MODER |= (1UL << GPIO_MODER_MODER5_Pos);  // PA5: general-purpose output (push-pull is default)

    while(1) {
        GPIOA->ODR |=  BIT_5; // LED on
        for (int i = 0; i < 200000; i++); // arbitrary delay
        GPIOA->ODR &= ~BIT_5; // LED off
        for (int i = 0; i < 400000; i++); // twice the arbitrary delay
    }
    return 0;
}
