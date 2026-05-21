#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led_strip.h>

#define NEOPIXEL_NODE DT_ALIAS(neopixel)

static const struct device *const neopixel = DEVICE_DT_GET(NEOPIXEL_NODE);

static const struct gpio_dt_spec buzzer = GPIO_DT_SPEC_GET(DT_ALIAS(buzzer), gpios);

static void neopixel_red(bool on)
{
    struct led_rgb pixel = {
        .r = on ? 255 : 0,
        .g = 0,
        .b = 0,
    };

    led_strip_update_rgb(neopixel, &pixel, 1);
}

static void buzzer_tone(uint32_t freq_hz, uint32_t duration_ms)
{
    uint32_t half_period_us = 1000000U / (freq_hz * 2U);
    uint32_t cycles = (duration_ms * 1000U) / (half_period_us * 2U);

    for (uint32_t i = 0; i < cycles; i++) {
        gpio_pin_set_dt(&buzzer, 1);
        k_busy_wait(half_period_us);

        gpio_pin_set_dt(&buzzer, 0);
        k_busy_wait(half_period_us);
    }
}

int main(void)
{
    gpio_pin_configure_dt(&buzzer, GPIO_OUTPUT_INACTIVE);

    bool red_on = false;
    int64_t next_led_toggle = k_uptime_get() + 1000;

    uint32_t freq = 500;
    int direction = 1;

    neopixel_red(false);

    while (1) {
        int64_t now = k_uptime_get();

        // blink neo pixel at 1 Hz
        if (now >= next_led_toggle) {
            red_on = !red_on;
            neopixel_red(red_on);
            next_led_toggle = now + 1000;
        }

        buzzer_tone(freq, 20);

        if (direction > 0) {
            freq += 25;
            if (freq >= 1500) {
                freq = 1500;
                direction = -1;
            }
        } else {
            freq -= 25;
            if (freq <= 500) {
                freq = 500;
                direction = 1;
            }
        }
    }
}