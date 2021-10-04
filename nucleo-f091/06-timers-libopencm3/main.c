/*
 * Test Timer 14 on Nucleo-F091 using libopencm3
 *
 * Toggle a pin in the ISR for TIM14 interrupt
 */

#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>

void config_clocks(void) {
    // Configure system clock, AHB clock, and APB clock for 48 MHz operation
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
    rcc_periph_clock_enable(RCC_GPIOA);

    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8);
    gpio_set_af(GPIOA, GPIO_AF0, GPIO8);    // AF #0 on PA8 is MCO

    // User LED on PA5
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);

    // Timer ISR toggles PA10
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO10);
}

void config_timer14(void) {
    // TIM14 is general-purpose auto-reload 16-bit upcounter
    // Counter is clocked by APB clock by default (freq = PCLK)
    rcc_periph_clock_enable(RCC_TIM14);
    timer_set_period(TIM14, 0x7fff); // set auto-reload value
    timer_set_prescaler(TIM14, 0);  // overflow freq = PCLK / (reload + 1)
    timer_enable_irq(TIM14, TIM_DIER_UIE); // enable interrupt for update events
}

// Non-core ISR names are in libopencm3/stm32/f0/nvic.h
void tim14_isr(void) {
    gpio_toggle(GPIOA, GPIO10);
    timer_clear_flag(TIM14, TIM_SR_UIF); // clear the update interrupt flag
}

int main(void) {
    config_clocks();
    config_1ms_tick();
    config_gpio();
    config_timer14();
    nvic_enable_irq(NVIC_TIM14_IRQ);
    timer_enable_counter(TIM14);
    while (1) {
        gpio_toggle(GPIOA, GPIO5);
        delay_ms(1000);
    }
    return 0;
}
