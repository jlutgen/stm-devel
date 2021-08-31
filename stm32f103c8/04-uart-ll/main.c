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
#include <stm32f1xx_ll_usart.h>
#include <stm32f1xx_ll_utils.h>

static char msg[80];

void usart_write(USART_TypeDef *USARTx, const char *s) {
    for (; *s != 0; ++s) {
        while(!LL_USART_IsActiveFlag_TXE(USARTx));
        LL_USART_TransmitData8(USARTx, *s);
    }
}

void usart_read(USART_TypeDef *USARTx, char *s, int len) {
    for (int i = 0; i < len; i++) {
        while(!LL_USART_IsActiveFlag_RXNE(USARTx));
        *s = LL_USART_ReceiveData8(USARTx);
        if (*s == '\r' || *s == '\n') {
            *(s + 1) = '\0';
            return;
        }
        ++s;
    }
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
    LL_GPIO_InitTypeDef info;

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);

    LL_GPIO_StructInit(&info);  // Fill in default values; this isn't strictly necessary,
                                // as we later reset all fields except info.Pull,
                                // which is used only when configuring inputs.

    // User LED
    info.Pin = LL_GPIO_PIN_13;
    info.Mode = LL_GPIO_MODE_OUTPUT;
    info.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    info.Speed = LL_GPIO_SPEED_FREQ_LOW;
    LL_GPIO_Init(GPIOC, &info);

    // Default alternate function on PA8 is MCO
    info.Pin = LL_GPIO_PIN_8;
    info.Mode = LL_GPIO_MODE_ALTERNATE;
    info.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    info.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    LL_GPIO_Init(GPIOA, &info);
	
    // Default alternate function on PA9/10 is USART1_TX/RX
    info.Pin = LL_GPIO_PIN_9;
    info.Mode = LL_GPIO_MODE_ALTERNATE;
    info.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    info.Speed = LL_GPIO_SPEED_FREQ_LOW;
    LL_GPIO_Init(GPIOA, &info);
    // PA10: floating input (default)
}

void uart_init(void) {
    LL_USART_InitTypeDef info;

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

    LL_USART_StructInit(&info);  // Fill in defaults: TX/RX enabled, 8N1, 9600 baud
    info.BaudRate = 115200;
    LL_USART_Init(USART1, &info);
    LL_USART_Enable(USART1);
}


int main(void) {
	clock_init();
    gpio_init();
    uart_init();

    usart_write(USART1, "Hello from STMF103 smart!\r\n");
    sprintf(msg, "GPIOA->CRH=0x%lx\r\n", GPIOA->CRH);
    usart_write(USART1, msg);

	while(1) {
        usart_read(USART1, msg, 80);
        usart_write(USART1, "received:");
        usart_write(USART1, msg);
        usart_write(USART1, "\r\n");
        LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);
	}
}
