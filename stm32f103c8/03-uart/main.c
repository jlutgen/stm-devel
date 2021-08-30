/**
  ******************************************************************************
  * @file    main.c
  *
  * Bare-metal UART
  * 
  * Smart board: user LED on PC13
  ******************************************************************************
*/

#include <stm32f1xx.h>
#include <stdio.h>

static char msg[80];

void usart_read(USART_TypeDef *usart, char* s, int len) {
    for (int i = 0; i < len; i++) {
        while ((usart->SR & USART_SR_RXNE) != USART_SR_RXNE)
            ;
            
        *s = (uint8_t)(usart->DR); // Receive data, clear flag
        if (*s == '\r' || *s == '\n') {
            *(s + 1) = '\0';
            return;
        }
        ++s;
    }
}

void usart_write(USART_TypeDef *usart, char* s) {
    while (*s) {
        usart->DR = *s++;
        // Wait for data to be shifted from TDR to tx shift register
        while (!(usart->SR & USART_SR_TXE))
            ;
    }
    // Wait for last data to be transmitted (transmission complete)
    // while (!(usart->SR & USART_SR_TC))
        // ;
}

void clock_init(void) {
    // Configure system clock for 64 MHz operation

	// HSION = 1 by default (internal 8 MHz RC oscillator)
	// and HSI is selected as system clock by default.
	// PLL is off and not ready (i.e., is unlocked) by default.

	// Set the PLL source and  multiplier (PREDIV is divide by 1 by default)
	// Default PLL source is HSI / 2
    RCC->CFGR |= RCC_CFGR_PLLMULL16;  // Yes, MULL (!)
    // Enable the PLL
    RCC->CR |= RCC_CR_PLLON;
    // Wait for PLL ready
    while (!(RCC->CR & RCC_CR_PLLRDY));

	FLASH->ACR |= FLASH_ACR_LATENCY; // One wait state needed if sysclk will be over 24 MHz
    // Select PLL as system clock
    RCC->CFGR |= RCC_CFGR_SW_PLL;

    // Wait until PLL is switched on
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    // Make system clock available on MCO signal
    RCC->CFGR |= RCC_CFGR_MCO_SYSCLK;
}

void gpio_init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;  // enable peripheral bus clock for Port C
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;  // enable peripheral bus clock for Port A

    GPIOC->CRH &= 0xFF0FFFFF;  // clear config bits for PC13
    GPIOC->CRH |= 0x00200000;  // PC13: general-purpose push-pull output, max speed 2 MHz
    
    // RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // Enable Port A (must do this before setting alternate function)
	// RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	// RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // Enable USART2 on AHB
    // RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    // Default alternate function on PA8 is MCO
    GPIOA->CRH &= ~(0xF << (0 * 4)); // clear CNF8 and MODE8
    GPIOA->CRH |= 0b1011 << (0 * 4); // Alternate function, high-speed
	
    // Default alternate function on PA9/10 is USART1_TX/RX
    GPIOA->CRH &= ~(0xF << (1 * 4)); // clear CNF9 and MODE9
    GPIOA->CRH |= 0b1010 << (1 * 4); // Alternate function, low-speed
    // PA10: floating input (default)
    // GPIOA->CRH &= ~(0xF << (2 * 4)); // clear CNF10 and MODE10
    // GPIOA->CRH |= 0b1010 << (2 * 4); // Alternate function, low-speed

	// GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODER2) | GPIO_MODER_MODER2_1; // alternate function mode for A2 and A3
	// GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODER3) | GPIO_MODER_MODER3_1;
	// GPIOA->AFR[0] |= 0x01 << GPIO_AFRL_AFSEL2_Pos; // alternate function 1 on A2 and A3 is USART2_TX/RX
	// GPIOA->AFR[0] |= 0x01 << GPIO_AFRL_AFSEL3_Pos;
}

void uart_init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN; // Enable peripheral clock for USART1

    // Baud rate = PCLK2 / (16 * div)
    // For 115200 baud rate and PCLK = 64 MHz, this gives div = 34.72 

    // 115200 baud, 8N1
	USART1->BRR = (34 << 4) | 12; // div = 34 + 12/16 = 34.75
 	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE; // Enable tx and rx, enable uart
}


int main(void) {
	clock_init();
    gpio_init();
    uart_init();

    usart_write(USART1, "Hello from STMF103 smart!\r\n");
    sprintf(msg, "GPIOA->CRH=0x%lx\r\n", GPIOA->CRH);
    usart_write(USART1, msg);
    // while (!(USART1->SR & USART_SR_RXNE)){
    //     GPIOC->ODR ^=  1 << 13; // toggle LED
    //     for (int i = 0; i < 300000; i++); // arbitrary delay
    // }
	while(1) {
        usart_read(USART1, msg, 80);
        usart_write(USART1, "received:");
        usart_write(USART1, msg);
        usart_write(USART1, "\r\n");
        // GPIOC->ODR ^= 1 << 13;
        // for (int i = 0; i < 500000; i++); // arbitrary delay
	}
}
