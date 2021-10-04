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

static char src[9] = "abcdefgh";
static char dst[9] = "XXXXXXXX";

// char src[BUF_SIZE];  // SRAM
// char dst[BUF_SIZE];  // SRAM

// const char src[RAM_SIZE / 2 - 1276];  // FLASH
// char dst[RAM_SIZE / 2 - 1276];        // SRAM

// char *src = (char *)GPIO_PORT_A_BASE; // GPIOA_MODER address
// char dst[RAM_SIZE / 2 - 1276];        // SRAM

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

void config_clocks(void) {
    // Configure system clock for 48 MHz operation
    rcc_clock_setup_in_hsi_out_48mhz();

    // Make system clock (divided by prediv) available on MCO signal
    RCC_CFGR |= RCC_CFGR_MCOPRE_DIV128;
    rcc_set_mco(RCC_CFGR_MCO_SYSCLK);
}

void config_1ms_tick(void) {
    // Configure SysTick for 1 ms interrupts
    systick_set_reload(rcc_ahb_frequency / 1000 - 1);
    systick_clear(); // set current value to 0
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_counter_enable();
}

void delay_ms(int ms) {
    systick_clear();  // reset current value to zero; this also clears COUNTFLAG
    while (ms) {
        if (systick_get_countflag()) {
            // SysTick timer has counted down to zero, so 1 ms has elapsed
            ms--;
        }
    }
}

void config_gpio(void) {
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

void config_usart(void) {
    rcc_periph_clock_enable(RCC_USART2);

    // Defaults: 8 data bits, 1 stop bit, no parity (8N1)

    usart_set_baudrate(USART2, 115200);
    usart_set_mode(USART2, USART_MODE_TX_RX);
    usart_enable(USART2);
}

void config_dma(void) {
    rcc_periph_clock_enable(RCC_DMA);

    // Defaults: read from "peripheral", 8 bit memory size, non-circular mode

    dma_enable_mem2mem_mode(DMA1 ,DMA_CHANNEL1);
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL1);
    dma_enable_peripheral_increment_mode(DMA1, DMA_CHANNEL1);
}

int main(void) {
    char msg[80];
    config_clocks();
    config_1ms_tick();
    config_gpio();
    config_usart();
    config_dma();
    sprintf(msg, "src @%p:%s\r\n", src, src);
    usart_write(USART2, msg);
    sprintf(msg, "dst @%p:%s\r\n", dst, dst);
    usart_write(USART2, msg);
    // sprintf(msg, "src @%p\r\n", src);
    // usart_write(USART2, msg);
    // sprintf(msg, "dst @%p\r\n", dst);
    // usart_write(USART2, msg);
    // Configure and start a DMA transfer
    dma_set_peripheral_address(DMA1, DMA_CHANNEL1, (uint32_t)src);
    dma_set_memory_address(DMA1, DMA_CHANNEL1, (uint32_t)dst);
    dma_set_number_of_data(DMA1, DMA_CHANNEL1, 9);
    // usart_write(USART2, "start...");
    dma_enable_channel(DMA1, DMA_CHANNEL1);  // Start the transfer
    gpio_set(GPIOA, GPIO5);
    // Wait for transfer complete
    while (!dma_get_interrupt_flag(DMA1, DMA_CHANNEL1, DMA_TCIF))
        ;
    gpio_clear(GPIOA, GPIO5);
    // usart_write(USART2, "done\r\n");

    sprintf(msg, "src @%p:%s\r\n", src, src);
    usart_write(USART2, msg);
    sprintf(msg, "dst @%p:%s\r\n", dst, dst);
    usart_write(USART2, msg);
    while (1) {
    }
    return 0;
}
