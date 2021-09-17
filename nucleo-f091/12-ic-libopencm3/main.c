/*
 * Measure PWM parameters using two input capture channels on TIM2
 */

#include <stdio.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/systick.h>

#define HCLK 48000000

char msg[40];

void usart_read(uint32_t usart, char* s, int len) {
    for (int i = 0; i < len; i++) {
        *s = (uint8_t)usart_recv_blocking(usart);
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

void config_uart(void) {
    rcc_periph_clock_enable(RCC_USART2);

    // Defaults: 8 data bits, 1 stop bit, no parity (8N1)

    usart_set_baudrate(USART2, 115200);
    usart_set_mode(USART2, USART_MODE_TX_RX);
    usart_enable(USART2);
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
    systick_clear();  // set current value to 0
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
    gpio_set_af(GPIOA, GPIO_AF0, GPIO8);  // AF #0 on PA8 is MCO

    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);
    gpio_set_af(GPIOA, GPIO_AF1, GPIO2);  // AF #1 on PA2 is USART_TX

    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO3);
    gpio_set_af(GPIOA, GPIO_AF1, GPIO3);  // AF #1 on PA3 is USART_RX

    // PWM input on PA1
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO1);
    gpio_set_af(GPIOA, GPIO_AF2, GPIO1);  // AF #1 on PA3 is USART_RX
}

void config_timer2(void) {
    // TIM2 is general-purpose auto-reload 16- or 32-bit counter.
    // Upcounter by default.
    // Counter is clocked by APB clock by default (freq = PCLK)
    // Clock tick freq = PCLK / (PSC + 1)
    // Overflow freq = (clock tick freq) / (reload + 1)
    // With reload = 0xffffffff, this gives ~107.4 s overflow period.

    // To measure PWM signal on TI2, we configure the timer so that a rising
    // edge on TI2 is captured in CCR2 and also triggers a counter reset, while
    // a falling edge on TI2 is captured in CCR1. Thus CCR1 gives the duty cycle
    // in ticks, and CCR2 the period.

    rcc_periph_clock_enable(RCC_TIM2);

    timer_set_period(TIM2, 0xffffffff);
    // Divide input clock by 1
    timer_set_prescaler(TIM2, 0);
    // Capture/compare channel 1 is input, connected to TI2
    timer_ic_set_input(TIM2, TIM_IC1, TIM_IC_IN_TI2);
    // Capture/compare channel 2 is input, connected to TI2
    timer_ic_set_input(TIM2, TIM_IC2, TIM_IC_IN_TI2);
    // Channel 1 sensitive to falling edge
    timer_ic_set_polarity(TIM2, TIM_IC1, TIM_IC_FALLING);
    // Enable input capture on channel 1
    timer_ic_enable(TIM2, TIM_IC1);
    // Enable input capture on channel 2
    timer_ic_enable(TIM2, TIM_IC2);
    // Slave mode: reset mode
    timer_slave_set_mode(TIM2, TIM_SMCR_SMS_RM);
    // Trigger select: TI2 filtered (rising edge of TI2)
    timer_slave_set_trigger(TIM2, TIM_SMCR_TS_TI2FP2);

    timer_enable_counter(TIM2);
}

int main(void) {
    uint32_t pwm_duty_cycle;
    uint32_t pwm_period;

    config_clocks();
    config_1ms_tick();
    config_gpio();
    config_uart();
    config_timer2();
    usart_write(USART2, "Ready\r\n");
    while (1) {
        // Wait for input capture readings to be ready
        while (!timer_get_flag(TIM2, TIM_SR_CC2IF))
            ;
        pwm_duty_cycle = TIM2_CCR1;
        pwm_period = TIM2_CCR2;  // Reading this register also clears interrupt
                                 // flag (CC2IF)
        sprintf(msg, "%lu %lu\r\n", pwm_duty_cycle, pwm_period);
        usart_write(USART2, msg);
        delay_ms(1000);
    }
    return 0;
}
