#include "fire_alarm.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>

#include <errno.h>

#define NEOPIXEL_NODE DT_ALIAS(neopixel)
#define BUZZER_NODE DT_ALIAS(buzzer)

#define ALARM_STACK_SIZE 768
#define ALARM_PRIORITY 8
#define LED_TOGGLE_MS 1000
#define TONE_DURATION_MS 20
#define MIN_FREQ_HZ 500
#define MAX_FREQ_HZ 1500
#define FREQ_STEP_HZ 25

static const struct device *const neopixel = DEVICE_DT_GET(NEOPIXEL_NODE);
static const struct gpio_dt_spec buzzer =
    GPIO_DT_SPEC_GET(BUZZER_NODE, gpios);

static K_SEM_DEFINE(alarm_start_sem, 0, 1);
static K_THREAD_STACK_DEFINE(alarm_stack, ALARM_STACK_SIZE);

static struct k_thread alarm_thread_data;
static atomic_t alarm_active;
static atomic_t alarm_flash_red;
static bool alarm_thread_started;
static bool buzzer_ready;
static bool neopixel_ready;

static void neopixel_red(bool on)
{
    if (!neopixel_ready) {
        return;
    }

    struct led_rgb pixel = {
        .r = on ? 255 : 0,
        .g = 0,
        .b = 0,
    };

    (void)led_strip_update_rgb(neopixel, &pixel, 1);
}

static void buzzer_tone(uint32_t freq_hz, uint32_t duration_ms)
{
    if (!buzzer_ready) {
        k_msleep(duration_ms);
        return;
    }

    uint32_t half_period_us = 1000000U / (freq_hz * 2U);
    uint32_t cycles = (duration_ms * 1000U) / (half_period_us * 2U);

    for (uint32_t i = 0; i < cycles; i++) {
        gpio_pin_set_dt(&buzzer, 1);
        k_busy_wait(half_period_us);

        gpio_pin_set_dt(&buzzer, 0);
        k_busy_wait(half_period_us);
    }
}

static void step_frequency(uint32_t *freq_hz, int *direction)
{
    if (*direction > 0) {
        *freq_hz += FREQ_STEP_HZ;
        if (*freq_hz >= MAX_FREQ_HZ) {
            *freq_hz = MAX_FREQ_HZ;
            *direction = -1;
        }
    } else {
        *freq_hz -= FREQ_STEP_HZ;
        if (*freq_hz <= MIN_FREQ_HZ) {
            *freq_hz = MIN_FREQ_HZ;
            *direction = 1;
        }
    }
}

static void alarm_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (1) {
        k_sem_take(&alarm_start_sem, K_FOREVER);

        bool red_on = false;
        int64_t next_led_toggle = k_uptime_get();
        uint32_t freq_hz = MIN_FREQ_HZ;
        int direction = 1;

        neopixel_red(false);

        while (atomic_get(&alarm_active) != 0) {
            if (atomic_get(&alarm_flash_red) != 0) {
                int64_t now = k_uptime_get();

                if (now >= next_led_toggle) {
                    red_on = !red_on;
                    neopixel_red(red_on);
                    next_led_toggle = now + LED_TOGGLE_MS;
                }
            } else if (red_on) {
                red_on = false;
                neopixel_red(false);
            }

            buzzer_tone(freq_hz, TONE_DURATION_MS);
            step_frequency(&freq_hz, &direction);
        }

        neopixel_red(false);
        if (buzzer_ready) {
            gpio_pin_set_dt(&buzzer, 0);
        }
    }
}

int fire_alarm_init(void)
{
    int ret;

    buzzer_ready = device_is_ready(buzzer.port);
    if (buzzer_ready) {
        ret = gpio_pin_configure_dt(&buzzer, GPIO_OUTPUT_INACTIVE);
        if (ret != 0) {
            printk("fire alarm: buzzer configure failed: %d\n", ret);
            buzzer_ready = false;
        }
    } else {
        printk("fire alarm: buzzer not ready\n");
    }

    neopixel_ready = device_is_ready(neopixel);
    if (neopixel_ready) {
        neopixel_red(false);
    } else {
        printk("fire alarm: NeoPixel not ready\n");
    }

    if (!buzzer_ready && !neopixel_ready) {
        return -ENODEV;
    }

    k_thread_create(&alarm_thread_data, alarm_stack, ALARM_STACK_SIZE,
                    alarm_thread, NULL, NULL, NULL,
                    ALARM_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&alarm_thread_data, "fire_alarm");
    alarm_thread_started = true;

    return 0;
}

void fire_alarm_trigger(bool occupied)
{
    if (!alarm_thread_started) {
        return;
    }

    if (!atomic_cas(&alarm_active, 0, 1)) {
        return;
    }

    atomic_set(&alarm_flash_red, occupied ? 1 : 0);
    printk("fire alarm triggered%s\n", occupied ? " (occupied)" : "");
    k_sem_give(&alarm_start_sem);
}

void fire_alarm_stop(void)
{
    if (atomic_cas(&alarm_active, 1, 0)) {
        atomic_set(&alarm_flash_red, 0);
        printk("fire alarm cleared\n");
    }
}
