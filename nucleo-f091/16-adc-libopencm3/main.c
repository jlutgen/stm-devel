/*
 * ADC
 *
 */

#include <stdio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/systick.h>

#define TEMP110_CAL_ADDR ((uint16_t*)((uint32_t)0x1FFFF7C2))
#define TEMP30_CAL_ADDR ((uint16_t*)((uint32_t)0x1FFFF7B8))
#define VDD_CALIB ((uint16_t)(330))
#define VDD_APPLI ((uint16_t)(300))

static char msg[80];

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

    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO4);
    gpio_set_af(GPIOA, GPIO_AF4, GPIO4);  // AF #4 on PA4 is TIM14_CH1

    // PA0: analog input (ADC_CH0)
    GPIOA_MODER |= GPIO_MODE(0, GPIO_MODE_ANALOG);
}

void config_uart(void) {
    rcc_periph_clock_enable(RCC_USART2);

    // Defaults: 8 data bits, 1 stop bit, no parity (8N1)

    usart_set_baudrate(USART2, 115200);
    usart_set_mode(USART2, USART_MODE_TX_RX);
    usart_enable(USART2);
}

void config_adc(void) {
    rcc_periph_clock_enable(RCC_ADC1);
    adc_power_on(ADC1);
    adc_set_clk_source(ADC1, ADC_CLKSOURCE_PCLK_DIV2);
    ADC1_CHSELR |= ADC_CHSELR_CHSEL(16);  // Select ch 16 (temperature sensor)
    ADC1_CHSELR |= ADC_CHSELR_CHSEL(0);   // ... and ch 0 (PA0/ADC_IN0)
    adc_enable_temperature_sensor();
    // Datasheet says min sampling time for temperature sensor
    // is 4 us, which is 80 ADC clock cycles (ADC clock: 20 MHz)
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_239DOT5);
}

int main(void) {
    int32_t temperature;

    config_clocks();
    config_1ms_tick();
    config_gpio();
    config_uart();
    config_adc();
    while (1) {
        // Start a sequence of ADC conversions, one for
        // each channel we have enabled
        ADC1_CR |= ADC_CR_ADSTART;

        // Channel 0
        while (!(ADC1_ISR & ADC_ISR_EOC))
            ;  // Wait for conversion to be completed
        sprintf(msg, "%ld\t", ADC1_DR);
        usart_write(USART2, msg);

        // Channel 16
        while (!(ADC1_ISR & ADC_ISR_EOC))
            ;  // Wait for conversion to be completed
        temperature = ADC1_DR;
        // Convert reading to degrees Celsius using formula from
        // user manual
        temperature = (((int32_t)ADC1_DR * VDD_APPLI / VDD_CALIB) -
                       (int32_t)*TEMP30_CAL_ADDR);
        temperature = temperature * (int32_t)(110 - 30);
        temperature /= (int32_t)(*TEMP110_CAL_ADDR - *TEMP30_CAL_ADDR);
        temperature += 30;
        sprintf(msg, "%ld\r\n", temperature);
        usart_write(USART2, msg);
        delay_ms(2000);
    }
    return 0;
}
