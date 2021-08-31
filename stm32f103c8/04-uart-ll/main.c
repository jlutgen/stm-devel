/**
  ******************************************************************************
  * @file    main.c
  *
  * UART using LL
  * 
  * Smart board: user LED on PC13
  ******************************************************************************
*/

#include <stm32f1xx.h>
#include <stdio.h>
#include <stm32f1xx_ll_bus.h>
#include <stm32f1xx_ll_gpio.h>
#include <stm32f1xx_ll_rcc.h>
#include <stm32f1xx_ll_utils.h>

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

    LL_UTILS_ClkInitTypeDef clk;
    LL_UTILS_PLLInitTypeDef pll;
    clk.AHBCLKDivider = LL_RCC_SYSCLK_DIV_1;
    clk.APB1CLKDivider = LL_RCC_APB1_DIV_1;
    clk.APB2CLKDivider = LL_RCC_APB2_DIV_1;
    pll.PLLMul = LL_RCC_PLL_MUL_16;
    pll.Prediv = LL_RCC_PREDIV_DIV_2;
    LL_PLL_ConfigSystemClock_HSI(&pll,  &clk);

    // Make system clock available on MCO signal
    LL_RCC_ConfigMCO(LL_RCC_MCO1SOURCE_SYSCLK);
}

void gpio_init(void) {
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);

    // User LED
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_13, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(GPIOC, LL_GPIO_PIN_13, LL_GPIO_OUTPUT_PUSHPULL);

    // Default alternate function on PA8 is MCO
    GPIOA->CRH &= ~(0xF << (0 * 4)); // clear CNF8 and MODE8
    GPIOA->CRH |= 0b1011 << (0 * 4); // Alternate function, high-speed
	
    // Default alternate function on PA9/10 is USART1_TX/RX
    GPIOA->CRH &= ~(0xF << (1 * 4)); // clear CNF9 and MODE9
    GPIOA->CRH |= 0b1010 << (1 * 4); // Alternate function, low-speed
    // PA10: floating input (default)
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
