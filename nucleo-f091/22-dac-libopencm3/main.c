/*
 * DAC
 */

#include <math.h>
#include <libopencm3/stm32/dac.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

#define HCLK 48000000
#define PCLK HCLK

// DDS constants
#define TWO_32 4294967296.0  // 2^32
#define SAMPLE_FREQ 100000
#define TIMER14_PERIOD (PCLK / SAMPLE_FREQ)
#define OUT_FREQ_1 440  // 440 Hz
#define OUT_FREQ_2 660

// Globals for Timer 14 ISR
volatile uint32_t dac_data_1, dac_data_2;  // output value

// DDS units:
volatile uint32_t phase_accum_1, phase_accum_2;
#define PHASE_INCR_1 ((uint32_t)(OUT_FREQ_1 * TWO_32 / SAMPLE_FREQ))
#define PHASE_INCR_2 ((uint32_t)(OUT_FREQ_2 * TWO_32 / SAMPLE_FREQ))

// DDS waveform tables
#define TABLE_SIZE 256
uint16_t sin_table[TABLE_SIZE];
uint16_t saw_table[TABLE_SIZE];

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

    // User LED on PA5: output (push-pull by default)
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);

    // PA4: DAC channel 1 output: analog
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO4);
}

void config_dac(void) {
    rcc_periph_clock_enable(RCC_DAC1);
    dac_enable(DAC1, DAC_CHANNEL1);
}

void config_timer14(void) {
    // TIM14 is general-purpose auto-reload 16-bit upcounter
    // Counter is clocked by APB clock by default (freq = PCLK)
    // Clock tick freq = PCLK / (PSC + 1)
    // Overflow freq = (clock tick freq) / (reload + 1)
    rcc_periph_clock_enable(RCC_TIM14);
    timer_set_period(TIM14, TIMER14_PERIOD); // set auto-reload value
    timer_set_prescaler(TIM14, 0);  // overflow freq = PCLK / (reload + 1)
    timer_enable_irq(TIM14, TIM_DIER_UIE); // enable interrupt for update events
    nvic_enable_irq(NVIC_TIM14_IRQ);
    timer_enable_counter(TIM14);
}

// Non-core ISR names are in libopencm3/stm32/f0/nvic.h
// Compute DDS phase, update both DAC channels.
void tim14_isr(void) {
    // DDS wave table lookup
    phase_accum_1 += PHASE_INCR_1;
    // phase_accum_2 += PHASE_INCR_2;
    dac_data_1 = sin_table[phase_accum_1 >> 24];
    // dac_data_2 = sin_table[phase_accum_2 >> 24];

    dac_load_data_buffer_single(DAC1, dac_data_1, DAC_ALIGN_RIGHT12, DAC_CHANNEL1);
    timer_clear_flag(TIM14, TIM_SR_UIF); // clear the update interrupt flag
}

void build_lookup_tables(void) {
    // Build the lookup tables
    // Scaled to generate values between 0 and 4095 for 12-bit DAC
    int i;
    for (i = 0; i < TABLE_SIZE; i++) {
        sin_table[i] = 2048 + 2047 * sin(i * 6.2831853 / TABLE_SIZE);
        saw_table[i] = 4095 * i / TABLE_SIZE;
    }
}

int main(void) {
    config_clocks();
    config_1ms_tick();
    config_gpio();
    config_timer14();
    build_lookup_tables();
    config_dac();
    while (1) {
        gpio_toggle(GPIOA, GPIO5);
        delay_ms(500);
    }
    return 0;
}
