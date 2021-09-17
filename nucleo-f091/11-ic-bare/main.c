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

void config_uart(void) {
    RCC_APB1ENR |= RCC_APB1ENR_USART2EN;  // Enable USART2 on APB

    // Defaults: 8 data bits, 1 stop bit, no parity (8N1)

    // Set baud rate to 115200
    USART2_BRR = 48000000 / 115200;
    // tx enable, rx enable, uart enable
    USART2_CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
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

    // User LED on PA5: output (push-pull by default)
    GPIOA_MODER |= GPIO_MODE(5, GPIO_MODE_OUTPUT);

    GPIOA_MODER |= GPIO_MODE(8, GPIO_MODE_AF);  // AF mode
    GPIOA_AFRH |= GPIO_AFR(8 - 8, GPIO_AF0);    // AF #0 on PA8 is MCO
    GPIOA_OSPEEDR |= GPIO_OSPEED(8, GPIO_OSPEED_100MHZ);

    GPIOA_MODER |= GPIO_MODE(2, GPIO_MODE_AF);  // AF mode
    GPIOA_AFRL |= GPIO_AFR(2, GPIO_AF1);        // AF #1 on PA2 is USART2_TX

    GPIOA_MODER |= GPIO_MODE(3, GPIO_MODE_AF);  // AF mode
    GPIOA_AFRL |= GPIO_AFR(3, GPIO_AF1);        // AF #1 on PA3 is USART2_RX

    // PWM input on PA1
    GPIOA_MODER |= GPIO_MODE(1, GPIO_MODE_AF);  // AF mode
    GPIOA_AFRL |= GPIO_AFR(1, GPIO_AF2);        // AF #2 on PA1 is TIM2_CH2
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
    RCC_APB1ENR |= RCC_APB1ENR_TIM2EN;    // Enable clock for TIM2
    TIM2_ARR = 0xffffffff;                // Auto-reload value
    TIM2_PSC = 0;                         // Divide input clock by 1
    TIM2_CCMR1 |= TIM_CCMR1_CC1S_IN_TI2;  // Capture/compare channel 1 is input,
                                          // connected to TI2
    TIM2_CCMR1 |= TIM_CCMR1_CC2S_IN_TI2;  // Capture/compare channel 2 is input,
                                          // connected to TI2
    TIM2_CCER |= TIM_CCER_CC1P;           // Channel 1 sensitive to falling edge
    TIM2_CCER |= TIM_CCER_CC1E;           // Enable input capture on channel 1
    TIM2_CCER |= TIM_CCER_CC2E;           // Enable input capture on channel 2
    TIM2_SMCR |= TIM_SMCR_SMS_RM;         // Slave mode: reset mode
    TIM2_SMCR |= TIM_SMCR_TS_TI2FP2;  // Trigger select: TI2 filtered (rising
                                      // edge of TI2)
    TIM2_CR1 |= TIM_CR1_CEN;          // Enable TIM2
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
        while (!(TIM2_SR & TIM_SR_CC2IF))
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
