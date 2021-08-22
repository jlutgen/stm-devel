/**
  ******************************************************************************
  * @file    main.c
  *
  * USART2 is connected to ST-LINK USB on Nucleo board
  ******************************************************************************
*/

#include "stm32f0xx.h"

static char msg[80];

void usart1_read(char *s, int len) {
    for (int i = 0; i < len; i++) {
        while((USART1->ISR & USART_ISR_RXNE) != USART_ISR_RXNE);
        *s = (uint8_t)(USART1->RDR); /* Receive data, clear flag */
        if (*s == '\r' || *s == '\n') {
            *(s + 1) = '\0';
            return;
        }
        ++s;
    }
}

void usart2_read(char *s, int len) {
    for (int i = 0; i < len; i++) {
        while((USART2->ISR & USART_ISR_RXNE) != USART_ISR_RXNE);
        *s = (uint8_t)(USART2->RDR); /* Receive data, clear flag */
        if (*s == '\r' || *s == '\n') {
            *(s + 1) = '\0';
            return;
        }
        ++s;
    }
}

void usart1_write(char *s) {
    while (*s) {
        USART1->TDR = *s;
        s++;
    }
    while (!(USART1->ISR & USART_ISR_TC)) ;
}

void usart2_write(char *s) {
    while (*s) {
        USART2->TDR = *s++;
        // Wait for data to be shifted from TDR to tx shift register
        while (!(USART2->ISR & USART_ISR_TXE)) ;
    }
    // Wait for last data to be transmitted (transmission complete)
    while (!(USART2->ISR & USART_ISR_TC)) ;
}

// user LED on A5
int main(void) {
	// Configure system clock for 48 MHz operation

	// HSION = 1 by default (internal 8 MHz RC oscillator)
	// and HSI is selected as system clock by default.
	// PLL is off and not ready (i.e., is unlocked) by default.

	// Set the PLL source and  multiplier (PREDIV is divide by 1 by default)
	RCC->CFGR |= RCC_CFGR_PLLSRC_HSI_PREDIV | RCC_CFGR_PLLMUL6;
    // Enable the PLL
    RCC->CR |= RCC_CR_PLLON;
    // Wait for PLL ready
    while (!(RCC->CR & RCC_CR_PLLRDY));

	FLASH->ACR |= FLASH_ACR_LATENCY; // one wait state needed if sysclk will be over 24 MHz!!!
    // Select PLL as system clock
    RCC->CFGR |= RCC_CFGR_SW_PLL;

    // Wait until PLL is switched on
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    // Make system clock (divided by 8) available on MCO signal
    RCC->CFGR |= RCC_CFGR_MCOPRE_DIV16;
    RCC->CFGR |= RCC_CFGR_MCO_SYSCLK;

	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // Enable Port A (must do this before setting alternate function)
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // Enable USART2 on AHB
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

	GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODER8) | GPIO_MODER_MODER8_1; // alternate function mode for A8
	GPIOA->AFR[1] |= 0x00 << GPIO_AFRH_AFSEL8_Pos; // alternate function 0 on A8 is MCO (clock output)

	GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODER2) | GPIO_MODER_MODER2_1; // alternate function mode for A2 and A3
	GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODER3) | GPIO_MODER_MODER3_1;
	GPIOA->AFR[0] |= 0x01 << GPIO_AFRL_AFSEL2_Pos; // alternate function 1 on A2 and A3 is USART2_TX/RX
	GPIOA->AFR[0] |= 0x01 << GPIO_AFRL_AFSEL3_Pos;

	GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODER6) | GPIO_MODER_MODER6_1; // alternate function mode for B6 and B7
	GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODER7) | GPIO_MODER_MODER7_1;
	GPIOB->AFR[0] |= 0x00 << GPIO_AFRL_AFSEL6_Pos; // alternate function 0 on B6 and B7 is USART1_TX/RX
	GPIOB->AFR[0] |= 0x00 << GPIO_AFRL_AFSEL7_Pos;

	GPIOA->MODER |= 0x01 << 10; // A5: output

	/* (1) Oversampling by 16, 115200 baud */
	/* (2) 8 data bit, 1 start bit, 1 stop bit, no parity */
	USART2->BRR = 48000000 / 115200; /* (1) */
	USART2->CR1 = USART_CR1_TE | USART_CR1_RXNEIE | USART_CR1_RE| USART_CR1_UE; /* (2) */

	/* (1) Oversampling by 16, 115200 baud */
	/* (2) 8 data bit, 1 start bit, 1 stop bit, no parity */
	USART1->BRR = 48000000 / 115200; /* (1) */
	USART1->CR1 = USART_CR1_TE | USART_CR1_RXNEIE | USART_CR1_RE| USART_CR1_UE; /* (2) */

    usart2_write("Hello from Nucleo!!\r\n");
	while(1) {
        usart2_read(msg, 80);
        usart2_write("received:");
        usart2_write(msg);
        usart2_write("\r\n");
        GPIOA->ODR ^= 1U << 5;
	}
}
