/*
 * Try the user button connected to PC13 on the Nucleo-F091RC
 */

#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

void config_clocks(void) {
    // Configure system clock, AHB clock, and APB clock for 48 MHz operation
    rcc_clock_setup_in_hsi_out_48mhz();
}

void config_gpio(void) {
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOC);

    // User LED on PA5: output (push-pull by default)
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);

    // User button on PC13: floating low-speed digital input (default)
}

int main(void) {
    config_clocks();
    config_gpio();
    while (1) {
        if (gpio_get(GPIOC, GPIO13)) // button not pressed
            gpio_clear(GPIOA, GPIO5); // LED off
        else
            gpio_set(GPIOA, GPIO5); // LED on
    }
    return 0;
}
