#include "remote_pico_interrupts.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>

#include <errno.h>

#include "../drivers/sensor/remote_pico/remote_pico.h"
#include "fire_alarm.h"
#include "remote_pico_display.h"

#define REMOTE_PICO_NODE DT_NODELABEL(remote_pico)
#define IRQ_CONFIG_RETRY_MS 2000

static const struct gpio_dt_spec remote_irq =
    GPIO_DT_SPEC_GET_OR(REMOTE_PICO_NODE, int_gpios, {0});

static const struct device *remote_sensor;
static struct gpio_callback remote_irq_cb;
static K_MUTEX_DEFINE(remote_sensor_lock);
static bool interrupt_demo_configured;
static uint32_t last_irq_config_try_ms;
static int32_t fire_threshold_centi_c =
    REMOTE_PICO_FIRE_THRESHOLD_DEFAULT_CENTI_C;

static void remote_irq_work_handler(struct k_work *work);
static K_WORK_DEFINE(remote_irq_work, remote_irq_work_handler);

static bool remote_sensor_is_occupied(void)
{
    struct sensor_value v;

    if (sensor_channel_get(remote_sensor,
                           (enum sensor_channel)REMOTE_PICO_CHAN_OCCUPANCY,
                           &v) != 0) {
        return false;
    }

    return v.val1 != 0;
}

void remote_pico_interrupts_lock_sensor(void)
{
    k_mutex_lock(&remote_sensor_lock, K_FOREVER);
}

void remote_pico_interrupts_unlock_sensor(void)
{
    k_mutex_unlock(&remote_sensor_lock);
}

static void remote_irq_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (remote_sensor == NULL) {
        return;
    }

    remote_pico_interrupts_lock_sensor();

    int ret = sensor_sample_fetch(remote_sensor);
    if (ret != 0) {
        printk("[%6u ms][irq] sample_fetch failed: %d\n",
               k_uptime_get_32(), ret);
        remote_pico_interrupts_unlock_sensor();
        return;
    }

    struct sensor_value v;

    ret = sensor_channel_get(remote_sensor,
                             (enum sensor_channel)REMOTE_PICO_CHAN_INT_SRC,
                             &v);
    if (ret == 0 && v.val1 != 0) {
        uint8_t src = (uint8_t)v.val1;

        printk("[%6u ms][irq] ", k_uptime_get_32());
        remote_pico_print_irq_sources(src);
        printk("  ");
        remote_pico_print_sensor_snapshot(remote_sensor);
        printk("\n");

        if (src & REMOTE_PICO_INT_T_OBJ_HIGH) {
            fire_alarm_trigger(remote_sensor_is_occupied());
        }

        ret = remote_pico_clear_interrupts(remote_sensor);
        if (ret != 0) {
            printk("[%6u ms][irq] clear failed: %d\n",
                   k_uptime_get_32(), ret);
        }
    }

    remote_pico_interrupts_unlock_sensor();
}

static void remote_irq_isr(const struct device *port,
                           struct gpio_callback *cb,
                           uint32_t pins)
{
    ARG_UNUSED(port);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    k_work_submit(&remote_irq_work);
}

static int configure_irq_gpio(void)
{
    if (remote_irq.port == NULL || !device_is_ready(remote_irq.port)) {
        printk("remote_pico interrupt GPIO not ready\n");
        return -ENODEV;
    }

    int ret = gpio_pin_configure_dt(&remote_irq, GPIO_INPUT);
    if (ret != 0) {
        printk("interrupt GPIO configure failed: %d\n", ret);
        return ret;
    }

    gpio_init_callback(&remote_irq_cb, remote_irq_isr, BIT(remote_irq.pin));

    ret = gpio_add_callback(remote_irq.port, &remote_irq_cb);
    if (ret != 0) {
        printk("interrupt GPIO callback failed: %d\n", ret);
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(&remote_irq, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret != 0) {
        printk("interrupt GPIO enable failed: %d\n", ret);
        return ret;
    }

    printk("GPIO%u interrupt input ready\n", remote_irq.pin);

    if (gpio_pin_get_dt(&remote_irq) > 0) {
        k_work_submit(&remote_irq_work);
    }

    return 0;
}

int remote_pico_interrupts_init(const struct device *sensor)
{
    remote_sensor = sensor;

    if (remote_sensor == NULL) {
        return -EINVAL;
    }

    return configure_irq_gpio();
}

int32_t remote_pico_interrupts_get_fire_threshold(void)
{
    return fire_threshold_centi_c;
}

int remote_pico_interrupts_set_fire_threshold(int32_t centi_c)
{
    int ret = 0;

    if (remote_sensor == NULL) {
        return -ENODEV;
    }

    remote_pico_interrupts_lock_sensor();

    ret = remote_pico_set_interrupts(remote_sensor, 0);
    if (ret == 0) {
        ret = remote_pico_set_object_temp_high(remote_sensor, centi_c);
        if (ret == 0) {
            fire_threshold_centi_c = centi_c;
        }
    }
    if (ret == 0) {
        ret = remote_pico_clear_interrupts(remote_sensor);
    }
    if (ret == 0) {
        ret = remote_pico_set_interrupts(remote_sensor,
                                         REMOTE_PICO_INT_T_OBJ_HIGH);
    }
    if (ret == 0) {
        interrupt_demo_configured = true;
    }

    remote_pico_interrupts_unlock_sensor();

    return ret;
}

static int configure_remote_interrupt_demo(const struct device *sensor)
{
    int ret;

    ret = remote_pico_set_interrupts(sensor, 0);
    if (ret != 0) {
        printk("interrupt demo: disable INT_EN failed: %d\n", ret);
        return ret;
    }

    ret = remote_pico_set_object_temp_high(sensor, fire_threshold_centi_c);
    if (ret != 0) {
        printk("interrupt demo: write T_obj threshold failed: %d\n", ret);
        return ret;
    }

    ret = remote_pico_clear_interrupts(sensor);
    if (ret != 0) {
        printk("interrupt demo: clear INT_SRC failed: %d\n", ret);
        return ret;
    }

    ret = remote_pico_set_interrupts(sensor, REMOTE_PICO_INT_T_OBJ_HIGH);
    if (ret != 0) {
        printk("interrupt demo: enable T_OBJ_HIGH failed: %d\n", ret);
        return ret;
    }

    printk("interrupt demo: GPIO16, T_obj > %d.%02d C\n",
           fire_threshold_centi_c / 100,
           fire_threshold_centi_c % 100);

    return 0;
}

void remote_pico_interrupts_try_configure(bool force)
{
    uint32_t now = k_uptime_get_32();

    if (remote_sensor == NULL || interrupt_demo_configured) {
        return;
    }

    if (!force && (now - last_irq_config_try_ms) < IRQ_CONFIG_RETRY_MS) {
        return;
    }

    last_irq_config_try_ms = now;

    int ret = configure_remote_interrupt_demo(remote_sensor);
    if (ret == 0) {
        interrupt_demo_configured = true;
        return;
    }

    printk("interrupt demo config pending: %d "
           "(check sensor-node firmware, power, I2C wiring)\n", ret);
}
