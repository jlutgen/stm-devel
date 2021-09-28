/*
 * RTC
 */

#include <stdio.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/systick.h>

static char msg[80];
static char* day_names[] = {
    "Monday", "Tuesday",  "Wednesday", "Thursday",
    "Friday", "Saturday", "Sunday",
};

struct datetime {
    int8_t yy;
    int8_t mo;
    int8_t dd;
    int8_t day_of_week;
    int8_t hh;
    int8_t mm;
    int8_t ss;
};

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
    // RCC_CFGR |= RCC_CFGR_MCOPRE_DIV128;
    // rcc_set_mco(RCC_CFGR_MCO_SYSCLK);

    // Make LSE available on MCO signal
    rcc_set_mco(RCC_CFGR_MCO_LSE);

    // Start the 32.768 kHz low-speed external oscillator (LSE)

    rcc_periph_clock_enable(RCC_PWR);
    // Allow access to RCC_BDCR bits
    pwr_disable_backup_domain_write_protect();
    rcc_osc_on(RCC_LSE);
    while (!rcc_is_osc_ready(RCC_LSE))
        ;
    rcc_set_rtc_clock_source(RCC_LSE);
    rcc_enable_rtc_clock();
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
    gpio_set_af(GPIOA, GPIO_AF1, GPIO2);  // AF #1 on PA2 is USART2_TX

    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO3);
    gpio_set_af(GPIOA, GPIO_AF1, GPIO3);  // AF #1 on PA3 is USART2_RX

    // User LED on PA5: output (push-pull by default)
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);
}

void config_usart(void) {
    rcc_periph_clock_enable(RCC_USART2);

    // Defaults: 8 data bits, 1 stop bit, no parity (8N1)

    usart_set_baudrate(USART2, 115200);
    usart_set_mode(USART2, USART_MODE_TX_RX);
    usart_enable(USART2);
}

void set_rtc_clock(void) {
    rtc_unlock();
    rtc_set_init_flag();
    rtc_wait_for_init_ready();
    // Program date: 2021-09-24
    // N.B. - The rtc_*_set_*() functions in libopencm3
    // use |= and &= to modify RTC_DR and RTC_TR, which does not
    // work properly unless we bypass the shadow registers.
    // See
    // https://community.st.com/s/question/0D53W000005sQrJ/problem-with-updating-rtctr-and-rtcdr-registers-in-stm32f098
    rtc_enable_bypass_shadow_register();
    rtc_calendar_set_date(99, 2, 12, RTC_DR_WDU_THU);
    rtc_disable_bypass_shadow_register();
    rtc_clear_init_flag();
    rtc_lock();
}

bool read_rtc_clock(struct datetime* dt) {
    uint32_t dr;
    uint32_t tr;
    if (!(RTC_ISR & RTC_ISR_RSF)) {
        return false;
    }
    dr = RTC_DR;
    tr = RTC_TR;
    dt->yy =
        10 * ((dr & (RTC_DR_YT_MASK << RTC_DR_YT_SHIFT)) >> RTC_DR_YT_SHIFT);
    dt->yy += ((dr & (RTC_DR_YU_MASK << RTC_DR_YU_SHIFT)) >> RTC_DR_YU_SHIFT);
    dt->day_of_week =
        ((dr & (RTC_DR_WDU_MASK << RTC_DR_WDU_SHIFT)) >> RTC_DR_WDU_SHIFT);
    dt->mo =
        10 * ((dr & (RTC_DR_MT_MASK << RTC_DR_MT_SHIFT)) >> RTC_DR_MT_SHIFT);
    dt->mo += ((dr & (RTC_DR_MU_MASK << RTC_DR_MU_SHIFT)) >> RTC_DR_MU_SHIFT);
    dt->dd =
        10 * ((dr & (RTC_DR_DT_MASK << RTC_DR_DT_SHIFT)) >> RTC_DR_DT_SHIFT);
    dt->dd += ((dr & (RTC_DR_DU_MASK << RTC_DR_DU_SHIFT)) >> RTC_DR_DU_SHIFT);
    dt->hh =
        10 * ((tr & (RTC_TR_HT_MASK << RTC_TR_HT_SHIFT)) >> RTC_TR_HT_SHIFT);
    dt->hh += ((tr & (RTC_TR_HU_MASK << RTC_TR_HU_SHIFT)) >> RTC_TR_HU_SHIFT);
    dt->mm =
        10 * ((tr & (RTC_TR_MNT_MASK << RTC_TR_MNT_SHIFT)) >> RTC_TR_MNT_SHIFT);
    dt->mm +=
        ((tr & (RTC_TR_MNU_MASK << RTC_TR_MNU_SHIFT)) >> RTC_TR_MNU_SHIFT);
    dt->ss =
        10 * ((tr & (RTC_TR_ST_MASK << RTC_TR_ST_SHIFT)) >> RTC_TR_ST_SHIFT);
    dt->ss += ((tr & (RTC_TR_SU_MASK << RTC_TR_SU_SHIFT)) >> RTC_TR_SU_SHIFT);
    return true;
}

int main(void) {
    struct datetime dt;

    config_clocks();
    config_1ms_tick();
    config_gpio();
    config_usart();
    set_rtc_clock();
    while (1) {
        if (!read_rtc_clock(&dt)) {
            usart_write(USART2, "RTC not yet synchronized\r\n");
            continue;
        }
        sprintf(msg, "%s 20%02u-%02d-%02d %02d:%02d:%02d\r\n",
                day_names[dt.day_of_week - 1], dt.yy, dt.mo, dt.dd, dt.hh,
                dt.mm, dt.ss);
        usart_write(USART2, msg);

        delay_ms(1000);
    }
    return 0;
}
