#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>

#include <errno.h>

#include "config_mode.h"
#include "fire_alarm.h"
#include "remote_pico_display.h"
#include "remote_pico_interrupts.h"
#include "../drivers/sensor/remote_pico/remote_pico.h"

/*
 * Base station application.
 *
 * Polls the sensor node every second through the Zephyr Sensor API.
 * GPIO16 is also wired to the sensor node interrupt output and handled by
 * remote_pico_interrupts.c.
 *
 * Output goes to UART0 via printk (picoprobe CDC-ACM on Pico 2).
 */

#define POLL_PERIOD K_SECONDS(1)
#define DEBOUNCE_MS 50
#define BUTTON_STACK_SIZE 1024
#define BUTTON_PRIORITY 5

enum base_station_mode {
    NORMAL_MODE,
    CONFIGURATION_MODE,
};

static const struct gpio_dt_spec button =
    GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);

static struct gpio_callback button_cb;
static struct k_sem btn_sem;
static struct k_thread button_thread_data;
static K_THREAD_STACK_DEFINE(button_stack, BUTTON_STACK_SIZE);
static atomic_t current_mode;
static uint32_t last_accepted_press_ms;

static int32_t sensor_value_to_centi_c(const struct sensor_value *v)
{
    return (v->val1 * 100) + (v->val2 / 10000);
}

static void update_fire_alarm_state(const struct device *sensor)
{
    struct sensor_value v;

    if (sensor_channel_get(sensor,
                           (enum sensor_channel)REMOTE_PICO_CHAN_OBJECT_TEMP,
                           &v) != 0) {
        return;
    }

    if (sensor_value_to_centi_c(&v)
        <= remote_pico_interrupts_get_fire_threshold()) {
        fire_alarm_stop();
    }
}

static bool in_configuration_mode(void)
{
    return atomic_get(&current_mode) == CONFIGURATION_MODE;
}

static bool configuration_mode_should_exit(void)
{
    return !in_configuration_mode();
}

static void set_mode(enum base_station_mode mode)
{
    atomic_set(&current_mode, mode);
    printk("Mode: %s\n",
           mode == CONFIGURATION_MODE ? "configuration" : "normal");
}

static void button_isr(const struct device *dev,
                       struct gpio_callback *cb,
                       uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    k_sem_give(&btn_sem);
}

static void button_task(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (1) {
        k_sem_take(&btn_sem, K_FOREVER);

        uint32_t now = k_uptime_get_32();
        if ((now - last_accepted_press_ms) < DEBOUNCE_MS) {
            continue;
        }
        last_accepted_press_ms = now;

        if (in_configuration_mode()) {
            set_mode(NORMAL_MODE);
        } else {
            set_mode(CONFIGURATION_MODE);
        }
    }
}

static int configure_mode_button(void)
{
    k_sem_init(&btn_sem, 0, 1);

    if (!gpio_is_ready_dt(&button)) {
        printk("mode button GPIO not ready\n");
        return -ENODEV;
    }

    int ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
    if (ret != 0) {
        printk("mode button configure failed: %d\n", ret);
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret != 0) {
        printk("mode button interrupt failed: %d\n", ret);
        return ret;
    }

    gpio_init_callback(&button_cb, button_isr, BIT(button.pin));

    ret = gpio_add_callback(button.port, &button_cb);
    if (ret != 0) {
        printk("mode button callback failed: %d\n", ret);
        return ret;
    }

    k_thread_create(&button_thread_data, button_stack, BUTTON_STACK_SIZE,
                    button_task, NULL, NULL, NULL,
                    BUTTON_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&button_thread_data, "mode_button");

    printk("GPIO%u mode button ready\n", button.pin);

    return 0;
}

int main(void)
{
    const struct device *sensor = DEVICE_DT_GET(DT_NODELABEL(remote_pico));

    printk("\n=== Base station starting ===\n");

    if (!device_is_ready(sensor)) {
        printk("remote_pico device not ready\n");
        return 0;
    }

    int ret = fire_alarm_init();
    if (ret != 0) {
        printk("continuing without local fire alarm outputs\n");
    }

    ret = configure_mode_button();
    if (ret != 0) {
        printk("continuing without configuration-mode button\n");
    }

    ret = remote_pico_interrupts_init(sensor);
    if (ret != 0) {
        printk("continuing with polling only\n");
    }

    remote_pico_interrupts_lock_sensor();
    remote_pico_interrupts_try_configure(true);
    remote_pico_interrupts_unlock_sensor();

    printk("remote_pico ready, polling every 1 s\n\n");

    while (1) {
        if (in_configuration_mode()) {
            config_mode_run(sensor, configuration_mode_should_exit);

            if (in_configuration_mode()) {
                set_mode(NORMAL_MODE);
            }
        }

        remote_pico_interrupts_lock_sensor();

        ret = sensor_sample_fetch(sensor);
        if (ret != 0) {
            printk("[%6u ms] sample_fetch failed: %d\n",
                   k_uptime_get_32(), ret);
            remote_pico_interrupts_unlock_sensor();
            k_sleep(POLL_PERIOD);
            continue;
        }

        remote_pico_interrupts_try_configure(false);
        update_fire_alarm_state(sensor);

        printk("[%6u ms] ", k_uptime_get_32());
        remote_pico_print_sensor_snapshot(sensor);
        printk("\n");

        remote_pico_interrupts_unlock_sensor();
        k_sleep(POLL_PERIOD);
    }

    return 0;
}
