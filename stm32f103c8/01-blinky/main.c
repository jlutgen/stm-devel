/*
 * Bare metal blinky for STM32F103C8
 *
 * Smart board: user LED on PC13
 */

#include <stdint.h>
#include <stm32f1xx.h>

#define BIT_13 (1UL<<13)

int main(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;  // enable peripheral bus clock for Port C
    GPIOC->CRH &= 0xFF0FFFFF;  // clear config bits for PC13
    GPIOC->CRH |= 0x00200000;  // PC13: general-purpose push-pull output, max speed 2 MHz
    while(1) {
        GPIOC->ODR ^=  BIT_13; // toggle LED
        for (int i = 0; i < 50000; i++); // arbitrary delay
    }
    return 0;
}
