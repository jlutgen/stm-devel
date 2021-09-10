/**
 ******************************************************************************
 * @file    04-uart-libopencm3
 *
 * USART2 is connected to ST-LINK USB on Nucleo board
 ******************************************************************************
 */

#include <stdio.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>

static char msg[80];

void usart_read(uint32_t usart, char* s, int len) {
    for (int i = 0; i < len; i++) {
        while ((USART_ISR(usart) & USART_ISR_RXNE) != USART_ISR_RXNE)
            ;
        *s = (uint8_t)(USART_RDR(usart)); /* Receive data, clear flag */
        if (*s == '\r' || *s == '\n') {
            *(s + 1) = '\0';
            return;
        }
        ++s;
    }
}

void usart_write(uint32_t usart, char* s) {
    while (*s) {
        USART_TDR(usart) = *s++;
        // Wait for data to be shifted from TDR to tx shift register
        while (!(USART_ISR(usart) & USART_ISR_TXE))
            ;
    }
    // Wait for last data to be transmitted (transmission complete)
    while (!(USART_ISR(usart) & USART_ISR_TC))
        ;
}

void clock_config(void) {
    // Configure system clock for 48 MHz operation
    rcc_clock_setup_in_hsi_out_48mhz();

    // Make system clock (divided by prediv) available on MCO signal
    RCC_CFGR |= RCC_CFGR_MCOPRE_DIV128;
    rcc_set_mco(RCC_CFGR_MCO_SYSCLK);
}

void gpio_config(void) {
    // Must enable a peripheral's clock before writing to its registers
    RCC_AHBENR |= RCC_AHBENR_GPIOAEN;  // Enable Port A clock

    GPIOA_MODER |= GPIO_MODE(8, GPIO_MODE_AF);  // AF mode
    GPIOA_AFRH |= GPIO_AFR(8 - 8, GPIO_AF0);    // AF #0 on PA8 is MCO

    GPIOA_MODER |= GPIO_MODE(2, GPIO_MODE_AF);  // AF mode
    GPIOA_AFRL |= GPIO_AFR(2, GPIO_AF1);        // AF #1 on PA2 is USART2_TX

    GPIOA_MODER |= GPIO_MODE(3, GPIO_MODE_AF);  // AF mode
    GPIOA_AFRL |= GPIO_AFR(3, GPIO_AF1);        // AF #1 on PA3 is USART2_RX

    // User LED on PA5: output (push-pull by default)
    GPIOA_MODER |= GPIO_MODE(5, GPIO_MODE_OUTPUT);
}

void uart_config(void) {
    RCC_APB1ENR |= RCC_APB1ENR_USART2EN;  // Enable USART2 on APB

    // Defaults: 8 data bits, 1 stop bit, no parity (8N1)

    // Set baud rate to 115200
    USART2_BRR = 48000000 / 115200;
    // tx enable, rx enable, uart enable
    USART2_CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

int main(void) {
    clock_config();
    gpio_config();
    uart_config();

    usart_write(USART2, "Hello from Nucleo!!\r\n");
    sprintf(msg, "sizeof(short): %d\r\n", sizeof(short));
    usart_write(USART2, msg);
    sprintf(msg, "sizeof(int): %d\r\n", sizeof(int));
    usart_write(USART2, msg);
    sprintf(msg, "sizeof(long int): %d\r\n", sizeof(int));
    usart_write(USART2, msg);
    while (1) {
        usart_read(USART2, msg, 80);
        usart_write(USART2, "received:");
        usart_write(USART2, msg);
        usart_write(USART2, "\r\n");
        GPIOA_ODR ^= 1 << 5;   // toggle user LED 
    }
}
