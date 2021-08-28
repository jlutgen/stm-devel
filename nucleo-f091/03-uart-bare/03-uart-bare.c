/**
 ******************************************************************************
 * @file    03-uart-bare.c
 *
 * USART2 is connected to ST-LINK USB on Nucleo board
 ******************************************************************************
 */

#include <stm32f0xx.h>
#include <stdio.h>

static char msg[80];

void usart_read(USART_TypeDef *usart, char* s, int len) {
    for (int i = 0; i < len; i++) {
        while ((usart->ISR & USART_ISR_RXNE) != USART_ISR_RXNE)
            ;
        *s = (uint8_t)(usart->RDR); /* Receive data, clear flag */
        if (*s == '\r' || *s == '\n') {
            *(s + 1) = '\0';
            return;
        }
        ++s;
    }
}

void usart_write(USART_TypeDef *usart, char* s) {
    while (*s) {
        usart->TDR = *s++;
        // Wait for data to be shifted from TDR to tx shift register
        while (!(usart->ISR & USART_ISR_TXE))
            ;
    }
    // Wait for last data to be transmitted (transmission complete)
    while (!(usart->ISR & USART_ISR_TC))
        ;
}

void clock_config(void) {
    // Configure system clock for 48 MHz operation

    // HSION = 1 by default (internal 8 MHz RC oscillator)
    // and HSI is selected as system clock by default.
    // PLL is off and not ready (i.e., is unlocked) by default.

    // Set the PLL source and  multiplier (PREDIV is divide by 1 by default)
    RCC->CFGR |= RCC_CFGR_PLLSRC_HSI_PREDIV | RCC_CFGR_PLLMUL6;
    // Enable the PLL
    RCC->CR |= RCC_CR_PLLON;
    // Wait for PLL ready
    while (!(RCC->CR & RCC_CR_PLLRDY))
        ;

    // One wait state needed if SYSCLK > 24 MHz
    FLASH->ACR |= FLASH_ACR_LATENCY;

    // Select PLL as system clock
    RCC->CFGR |= RCC_CFGR_SW_PLL;

    // Wait until PLL is switched on
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
        ;

    // Make system clock (divided by prediv) available on MCO signal
    RCC->CFGR |= RCC_CFGR_MCOPRE_DIV128;
    RCC->CFGR |= RCC_CFGR_MCO_SYSCLK;
}

void gpio_config(void) {
    // Must enable a peripheral's clock before writing to its registers
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;  // Enable Port A clock
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;  // Enable Port B clock

    GPIOA->MODER |= GPIO_MODER_MODER8_1;        // AF mode
    GPIOA->AFR[1] |= 0 << GPIO_AFRH_AFSEL8_Pos; // AF #0 on PA8 is MCO

    GPIOA->MODER |= GPIO_MODER_MODER2_1;        // AF mode
    GPIOA->AFR[0] |= 1 << GPIO_AFRL_AFSEL2_Pos; // AF #1 on PA2 is USART2_TX

    GPIOA->MODER |= GPIO_MODER_MODER3_1;        // AF mode
    GPIOA->AFR[0] |= 1 << GPIO_AFRL_AFSEL3_Pos; // AF #1 on PA3 is USART2_RX

    GPIOB->MODER |= GPIO_MODER_MODER6_1;        // AF mode
    GPIOB->AFR[0] |= 0 << GPIO_AFRL_AFSEL6_Pos; // AF #0 on PB3 is USART1_RX
    GPIOA->MODER |= 1 << (2 * 2); // (push-pull) output

    GPIOB->MODER |= GPIO_MODER_MODER7_1;        // AF mode
    GPIOB->AFR[0] |= 0 << GPIO_AFRL_AFSEL7_Pos; // AF #1 on PB3 is USART1_RX

    // User LED on PA5: push-pull output
    GPIOA->MODER |= 1 << (5 * 2);
}

void uart_config(void) {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;  // Enable USART2 on APB
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;  // Enable USART1 on APB

    // Defaults: 8 data bits, 1 stop bit, no parity (8N1)

    // Set baud rate to 115200
    USART1->BRR = 48000000 / 115200;
    // tx enable, rx enable, uart enable
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

    // Set baud rate to 115200
    USART2->BRR = 48000000 / 115200;  // 115200 baud rate
    // tx enable, rx enable, uart enable
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

int main(void) {
    clock_config();
    gpio_config();
    uart_config();

    usart_write(USART2, "Hello from Nucleo!!\r\n");
    sprintf(msg, "%lu\r\n", SystemCoreClock);
    usart_write(USART2, msg);
    SystemCoreClockUpdate();
    sprintf(msg, "%lu\r\n", SystemCoreClock);
    usart_write(USART2, msg);
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
        GPIOA->ODR ^= 1 << 5;   // toggle user LED 
    }
}
