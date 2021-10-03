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

    GPIOA_MODER |= GPIO_MODE(8, GPIO_MODE_AF);  // AF mode
    GPIOA_AFRH |= GPIO_AFR(8 - 8, GPIO_AF0);    // AF #0 on PA8 is MCO
    GPIOA_OSPEEDR |= GPIO_OSPEED(8, GPIO_OSPEED_100MHZ);

    // PA4: DAC channel 1 output: analog
    GPIOA_MODER |= GPIO_MODE(4, GPIO_MODE_ANALOG);
}

void config_dac(void) {
    RCC_APB1ENR |= RCC_APB1ENR_DACEN;
    DAC_CR(DAC1) |= DAC_CR_EN1;  // Enable DAC channel 1
}

void config_timer14(void) {
    // TIM14 is general-purpose auto-reload 16-bit upcounter
    // Counter is clocked by APB clock by default (freq = PCLK)
    // Clock tick freq = PCLK / (PSC + 1)
    // Overflow freq = (clock tick freq) / (reload + 1)
    RCC_APB1ENR |= RCC_APB1ENR_TIM14EN;  // enable clock for TIM14
    TIM14_ARR = TIMER14_PERIOD;           // auto-reload value
    TIM14_PSC = 0;                       // divide input clock by 1
    TIM14_DIER |=
        TIM_DIER_UIE;  // enable interrupts for update event (timer overflow)
    NVIC_ISER(0) |= (1 << NVIC_TIM14_IRQ);  // enable TIM14 interrupt
    TIM14_CR1 |= TIM_CR1_CEN;               // start TIM14
}

// Non-core ISR names are in libopencm3/stm32/f0/nvic.h
// Compute DDS phase, update both DAC channels.
void tim14_isr(void) {
    // DDS wave table lookup
    phase_accum_1 += PHASE_INCR_1;
    // phase_accum_2 += PHASE_INCR_2;
    dac_data_1 = sin_table[phase_accum_1 >> 24];
    // dac_data_2 = sin_table[phase_accum_2 >> 24];

    DAC_DHR12R1(DAC1) = dac_data_1; // 12-bit right-aligned value to output
    TIM14_SR &= ~TIM_SR_UIF;  // Clear the update interrupt flag
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
        GPIOA_ODR ^= (1 << 5);
        delay_ms(500);
    }
    return 0;
}
