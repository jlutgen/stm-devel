/*
 * DMA
 */

#include <stdio.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/systick.h>

#define HCLK 48000000

// static char msg[80];

#define RAM_SIZE (32 * 1024)
#define BUF_SIZE (RAM_SIZE / 2 - 1276)

// static char dst[9] = "XXXXXXXX";
// static char src[9] = "abcdefgh";

char src[BUF_SIZE];  // SRAM
char dst[BUF_SIZE];  // SRAM

// const char src[RAM_SIZE / 2 - 1276];  // FLASH
// char dst[RAM_SIZE / 2 - 1276];        // SRAM

// char *src = (char *)GPIO_PORT_A_BASE; // GPIOA_MODER address
// char dst[RAM_SIZE / 2 - 1276];        // SRAM

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

void config_clocks(void) {
    // Configure system clock for 48 MHz operation

    // HSION = 1 by default (internal 8 MHz RC oscillator)
    // and HSI is selected as system clock by default.
    // PLL is off and not ready (i.e., is unlocked) by default.

    // Set the PLL source and  multiplier
    // Constants aren't defined in an entirely consistent way in libopencm3
    // header files; for example, we must shift RCC_CFGR_PLLSRS_HSI_CLK_DIV2 to
    // the correct position ourselves, but RCC_CFGR_PLLMUL_MUL12 incorporates
    // the correct shift already.
    RCC_CFGR |= (RCC_CFGR_PLLSRC_HSI_CLK_DIV2 << 16) | RCC_CFGR_PLLMUL_MUL12;
    // Enable the PLL
    RCC_CR |= RCC_CR_PLLON;
    // Wait for PLL ready
    while (!(RCC_CR & RCC_CR_PLLRDY))
        ;

    // One wait state needed if SYSCLK > 24 MHz
    FLASH_ACR |= FLASH_ACR_LATENCY_1WS;

    // Select PLL as system clock
    RCC_CFGR |= RCC_CFGR_SW_PLL;

    // Wait until PLL is switched on
    while ((RCC_CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
        ;

    // Make system clock (divided by prediv) available on MCO signal
    RCC_CFGR |= RCC_CFGR_MCOPRE_DIV128;
    RCC_CFGR |= RCC_CFGR_MCO_SYSCLK << RCC_CFGR_MCO_SHIFT;
}

void config_1ms_tick(void) {
    // Configure SysTick for 1 ms interrupts

    STK_RVR = HCLK / 1000 - 1;  // reload value
    STK_CVR = 0;                // current value
    // Use processor clock, and enable
    STK_CSR = STK_CSR_CLKSOURCE_AHB | STK_CSR_ENABLE;
}

void delay_ms(int ms) {
    STK_CVR = 0;  // reset to zero; also, writing to CVR clears COUNTFLAG
    while (ms) {
        if (STK_CSR & STK_CSR_COUNTFLAG) {
            // SysTick timer has counted down to zero, so 1 ms has elapsed
            ms--;
        }
    }
}

void config_gpio(void) {
    // Must enable a peripheral's clock before writing to its registers
    RCC_AHBENR |= RCC_AHBENR_GPIOAEN;  // Enable Port A clock

    // PA5 (LED): general-purpose output (push-pull is default)
    GPIOA_MODER |= GPIO_MODE(5, GPIO_MODE_OUTPUT);

    GPIOA_MODER |= GPIO_MODE(2, GPIO_MODE_AF);  // AF mode
    GPIOA_AFRL |= GPIO_AFR(2, GPIO_AF1);        // AF #1 on PA2 is USART2_TX

    GPIOA_MODER |= GPIO_MODE(3, GPIO_MODE_AF);  // AF mode
    GPIOA_AFRL |= GPIO_AFR(3, GPIO_AF1);        // AF #1 on PA3 is USART2_RX

    GPIOA_MODER |= GPIO_MODE(8, GPIO_MODE_AF);  // AF mode
    GPIOA_AFRH |= GPIO_AFR(8 - 8, GPIO_AF0);    // AF #0 on PA8 is MCO
    GPIOA_OSPEEDR |= GPIO_OSPEED(8, GPIO_OSPEED_100MHZ);
}

void config_usart(void) {
    RCC_APB1ENR |= RCC_APB1ENR_USART2EN;  // Enable USART2 on APB

    // Defaults: 8 data bits, 1 stop bit, no parity (8N1)

    // Set baud rate to 115200
    USART2_BRR = 48000000 / 115200;
    // tx enable, rx enable, uart enable
    USART2_CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

void config_dma(void) {
    RCC_AHBENR |= RCC_AHBENR_DMA1EN;

    // Defaults: read from "peripheral", 8 bit memory size, non-circular mode

    // Use channel 1, memory-to-memory mode
    DMA1_CCR1 |= DMA_CCR_MEM2MEM;
    // Increment memory and "peripheral" addresses after each transfer
    DMA1_CCR1 |= DMA_CCR_MINC | DMA_CCR_PINC;
    // DMA1_CCR1 |= DMA_CCR_MINC;
    DMA1_CCR1 |= DMA_CCR_MSIZE_32BIT;
}

int main(void) {
    char msg[80];
    config_clocks();
    config_1ms_tick();
    config_gpio();
    config_usart();
    config_dma();
    // sprintf(msg, "src @%p:%s\r\n", src, src);
    // usart_write(USART2, msg);
    // sprintf(msg, "dst @%p:%s\r\n", dst, dst);
    // usart_write(USART2, msg);
    sprintf(msg, "src @%p\r\n", src);
    usart_write(USART2, msg);
    sprintf(msg, "dst @%p\r\n", dst);
    usart_write(USART2, msg);
    // Configure and start a DMA transfer
    DMA1_CPAR1 = (uint32_t)src;
    DMA1_CMAR1 = (uint32_t)dst;
    // DMA1_CNDTR1 = BUF_SIZE;  // Data length in bytes
    DMA1_CNDTR1 = BUF_SIZE / 4;  // Data length in 32-bit words
    usart_write(USART2, "start...");
    DMA1_CCR1 |= DMA_CCR_EN;  // Start the transfer
    GPIOA_ODR |= (1 << 5);    // PA5 high
    while (!(DMA1_ISR & DMA_ISR_TCIF1))
        ;                    // Wait for transfer complete
    GPIOA_ODR &= ~(1 << 5);  // PA5 low
    usart_write(USART2, "done\r\n");

    // sprintf(msg, "src @%p:%s\r\n", src, src);
    // usart_write(USART2, msg);
    // sprintf(msg, "dst @%p:%s\r\n", dst, dst);
    // usart_write(USART2, msg);
    while (1) {
    }
    return 0;
}
