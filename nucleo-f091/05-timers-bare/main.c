/*
 * Test Timer 14 on Nucleo-F091
 *
 * Toggle a pin in the ISR for TIM14 interrupt
 */

#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>

#define HCLK 48000000

void config_clocks(void) {
    // Configure system clock for 48 MHz operation

    // HSION = 1 by default (internal 8 MHz RC oscillator)
    // and HSI is selected as system clock by default.
    // PLL is off and not ready (i.e., is unlocked) by default.

    // Set the PLL source and  multiplier
    // Constants aren't defined in an entirely consistent way in libopencm3 header
    // files; for example, we must shift RCC_CFGR_PLLSRS_HSI_CLK_DIV2 to the correct
    // position ourselves, but RCC_CFGR_PLLMUL_MUL12 incorporates the correct shift
    // already.
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

    STK_RVR = HCLK / 1000 - 1; // reload value
    STK_CVR = 0;               // current value
    // Use processor clock, and enable
    STK_CSR = STK_CSR_CLKSOURCE | STK_CSR_ENABLE;
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

    // Timer ISR toggles PA10
    GPIOA_MODER |= GPIO_MODE(10, GPIO_MODE_OUTPUT);
}

void config_timer14(void) {
    // TIM14 is general-purpose auto-reload 16-bit upcounter
    // Counter is clocked by APB clock by default (freq = PCLK)
    // Clock tick freq = PCLK / (PSC + 1)
    // Overflow freq = (clock tick freq) / (reload + 1)
    RCC_APB1ENR |= RCC_APB1ENR_TIM14EN;  // enable clock for TIM14
    TIM14_ARR = 0x7fff;                  // auto-reload value
    TIM14_PSC = 0;                       // divide input clock by 1
    TIM14_DIER |=
        TIM_DIER_UIE;  // enable interrupts for update event (timer overflow)
}

// Non-core ISR names are in libopencm3/stm32/f0/nvic.h
void tim14_isr(void) {
    GPIOA_ODR ^= (1 << 10);   // toggle PA10
    TIM14_SR &= ~TIM_SR_UIF;  // clear the update interrupt flag
}

int main(void) {
    config_clocks();
    config_1ms_tick();
    config_gpio();
    config_timer14();
    nvic_enable_irq(NVIC_TIM14_IRQ);
    TIM14_CR1 |= TIM_CR1_CEN;  // start TIM14
    while (1) {
        // Toggle user LED on PA5
        GPIOA_ODR ^= (1 << 5);
        delay_ms(200);
    }
    return 0;
}
