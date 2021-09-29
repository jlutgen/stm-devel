/**
 ******************************************************************************
 * @file    04-uart-libopencm3
 *
 * USART2 is connected to ST-LINK USB on Nucleo board
 ******************************************************************************
 */

#include <stdio.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>

static char msg[80];

int _write(int file, const char *ptr, ssize_t len) {
    // Keep i defined outside the loop so we can return it
    int i;
    for (i = 0; i < len; i++) {
        // If we get a newline character, also be sure to send the carriage
        // return character first, otherwise the serial console may not
        // actually return to the left.
        if (ptr[i] == '\n') {
            usart_send_blocking(USART2, '\r');
        }

        // Write the character to send to the USART1 transmit buffer, and block
        // until it has been sent.
        usart_send_blocking(USART2, ptr[i]);
    }

    // Return the number of bytes we sent
    return i;
}

void usart_read(uint32_t usart, char* s, int len) {
    for (int i = 0; i < len; i++) {
        *s = (uint8_t) usart_recv_blocking(usart);
        if (*s == '\r' || *s == '\n') {
            *(s + 1) = '\0';
            return;
        }
        ++s;
    }
}

void usart_write(uint32_t usart, char* s) {
    while (*s) {
        usart_send_blocking(usart, *s++);
    }
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
    rcc_periph_clock_enable(RCC_GPIOA);

    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8);
    gpio_set_af(GPIOA, GPIO_AF0, GPIO8);    // AF #0 on PA8 is MCO

    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);
    gpio_set_af(GPIOA, GPIO_AF1, GPIO2);    // AF #1 on PA2 is USART2_TX

    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO3);
    gpio_set_af(GPIOA, GPIO_AF1, GPIO3);    // AF #1 on PA3 is USART2_RX

    // User LED on PA5: output (push-pull by default)
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);
}

void uart_config(void) {
    rcc_periph_clock_enable(RCC_USART2);

    // Defaults: 8 data bits, 1 stop bit, no parity (8N1)

    usart_set_baudrate(USART2, 115200);
    usart_set_mode(USART2, USART_MODE_TX_RX);
    usart_enable(USART2);
}

int main(void) {
    clock_config();
    gpio_config();
    uart_config();

    usart_write(USART2, "Hello from Nucleo!!\r\n");
    printf("sizeof(short): %d\n", sizeof(short));
    // sprintf(msg, "sizeof(short): %d\r\n", sizeof(short));
    // usart_write(USART2, msg);
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
