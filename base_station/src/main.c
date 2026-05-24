#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>

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

    if (sensor_value_to_centi_c(&v) <= REMOTE_PICO_FIRE_THRESHOLD_CENTI_C) {
        fire_alarm_stop();
    }
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

    ret = remote_pico_interrupts_init(sensor);
    if (ret != 0) {
        printk("continuing with polling only\n");
    }

    remote_pico_interrupts_lock_sensor();
    remote_pico_interrupts_try_configure(true);
    remote_pico_interrupts_unlock_sensor();

    printk("remote_pico ready, polling every 1 s\n\n");

    while (1) {
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
