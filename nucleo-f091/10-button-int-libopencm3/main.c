/*
 * Generate external interrupt using the user button
 * connected to PC13 on the Nucleo-F091RC
 */

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/syscfg.h>

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

void config_interrupts(void) {
    // Enable SYSCFG peripheral
    rcc_periph_clock_enable(RCC_SYSCFG_COMP);

    // Enable SYSCFG peripheral (alternative method)
    // rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_SYSCFGCOMPEN);

    // Connect external interrupt line 13 to PC13
    exti_select_source(EXTI13, GPIOC);

    // Unmask external interrupt line 13
    exti_enable_request(EXTI13);

    // Enable rising trigger and falling trigger
    exti_set_trigger(EXTI13, EXTI_TRIGGER_BOTH);

    // Enable external interrupts 4..15
    nvic_enable_irq(NVIC_EXTI4_15_IRQ);
}

void exti4_15_isr(void) {
    exti_reset_request(EXTI13);
    gpio_toggle(GPIOA, GPIO5);
}

int main(void) {
    config_clocks();
    config_gpio();
    config_interrupts();
    while (1) {
    }
    return 0;
}
