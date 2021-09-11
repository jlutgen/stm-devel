/*
 * Blinky for Nucleo-F091 using OpenCM3 library
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#define RCC_LED RCC_GPIOA
#define PORT_LED GPIOA
#define PIN_LED GPIO5

int main(void) {
    rcc_periph_clock_enable(RCC_LED);
    gpio_mode_setup(PORT_LED, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN_LED);
    gpio_set(PORT_LED, PIN_LED);
    while (1) {
        // Wait a little while
        for (int i = 0; i < 800000; i++) {
            __asm__("nop");
        }
        gpio_toggle(PORT_LED, PIN_LED);
    }
}
