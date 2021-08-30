/**
  ******************************************************************************
  * @file    main.c
  *
  * User LED is on PA5
  * USART2 is connected to ST-LINK USB on Nucleo board
  ******************************************************************************
*/

#include <stdio.h>
#include "stm32f0xx.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_usart.h"
#include "stm32f0xx_ll_utils.h"

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

void clock_config(void) {
	// Configure system clock for 48 MHz operation

	// HSION = 1 by default (internal 8 MHz RC oscillator)
	// and HSI is selected as system clock by default.
	// PLL is off and not ready (i.e., is unlocked) by default.
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI,
                                LL_RCC_PLL_MUL_6,
                                LL_RCC_PREDIV_DIV_1);
    LL_RCC_PLL_Enable();
    while (!LL_RCC_PLL_IsReady());

    LL_FLASH_SetLatency(LL_FLASH_LATENCY_1); // One wait state needed if SYSCLK > 24 MHz
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);

    // Make system clock (divided by 16) available on MCO signal
    LL_RCC_ConfigMCO(LL_RCC_MCO1SOURCE_SYSCLK, LL_RCC_MCO1_DIV_16);
}

void gpio_config(void) {
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA | LL_AHB1_GRP1_PERIPH_GPIOB);

	// Alternate function 0 on A8 is MCO (clock output)
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_8, LL_GPIO_AF_0);

	// Alternate function 1 on A2 and A3 is USART2_TX/RX
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_3, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_2, LL_GPIO_AF_1);
    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_3, LL_GPIO_AF_1);

    // Alternate function 0 on B6 and B7 is USART1_TX/RX
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_6, LL_GPIO_AF_0);
    LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_7, LL_GPIO_AF_0);

    // User LED on PA5: output (push-pull by default)
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
}

void uart_config(void) {
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1);

	// Oversampling by 16, 115200 baud
	// 8 data bits, 1 start bit, 1 stop bit, no parity
    LL_USART_SetBaudRate(USART2, 48000000, LL_USART_OVERSAMPLING_16, 115200);
    LL_USART_SetTransferDirection(USART2, LL_USART_DIRECTION_TX_RX);
    LL_USART_Enable(USART2);

	// Oversampling by 16, 115200 baud
	// 8 data bits, 1 start bit, 1 stop bit, no parity
    LL_USART_SetBaudRate(USART1, 48000000, LL_USART_OVERSAMPLING_16, 115200);
    LL_USART_SetTransferDirection(USART1, LL_USART_DIRECTION_TX_RX);
    LL_USART_Enable(USART1);
}

int main(void) {
    clock_config();
    gpio_config();
    uart_config();

    LL_RCC_ClocksTypeDef clocks;
    LL_RCC_GetSystemClocksFreq(&clocks);
    sprintf(msg, "SYSCLK=%lu\r\nPCLK1=%lu\r\nHCLK=%lu\r\n",
            clocks.SYSCLK_Frequency,
            clocks.PCLK1_Frequency,
            clocks.HCLK_Frequency);
    usart_write(USART2, msg);
	while(1) {
        usart_read(USART2, msg, 80);
        usart_write(USART2, "received:");
        usart_write(USART2, msg);
        usart_write(USART2, "\r\n");
        LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_5);
	}
}
