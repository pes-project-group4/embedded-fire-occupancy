#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>

#include "../drivers/sensor/remote_pico/remote_pico.h"

/*
 * Base station application.
 *
 * Polls the sensor node every second through the Zephyr Sensor API.
 * One sample_fetch() triggers an I2C burst read on the driver side;
 * subsequent channel_get() calls just decode from the cached buffer,
 * so they're cheap.
 *
 * Output goes to UART0 via printk (picoprobe CDC-ACM on Pico 2).
 */

#define POLL_PERIOD K_SECONDS(1)

static void print_temp(const char *label, const struct sensor_value *v)
{
    /* sensor_value::val2 is signed microunits; preserve sign cleanly. */
    int frac = v->val2 / 10000;
    if (frac < 0) frac = -frac;
    printk("%s=%d.%02d C  ", label, v->val1, frac);
}

static void print_humidity(const struct sensor_value *v)
{
    int frac = v->val2 / 1000;
    if (frac < 0) frac = -frac;
    printk("RH=%d.%03d %%  ", v->val1, frac);
}

int main(void)
{
    const struct device *sensor = DEVICE_DT_GET(DT_NODELABEL(remote_pico));

    printk("\n=== Base station starting ===\n");

    if (!device_is_ready(sensor)) {
        printk("remote_pico device not ready\n");
        return 0;
    }

    printk("remote_pico ready, polling every 1 s\n\n");

    while (1) {
        struct sensor_value v;
        int ret;

        ret = sensor_sample_fetch(sensor);
        if (ret != 0) {
            printk("[%6u ms] sample_fetch failed: %d\n",
                   k_uptime_get_32(), ret);
            k_sleep(POLL_PERIOD);
            continue;
        }

        printk("[%6u ms] ", k_uptime_get_32());

        /* --- BME680: air temp, humidity, gas --- */
        if (sensor_channel_get(sensor, SENSOR_CHAN_AMBIENT_TEMP, &v) == 0) {
            print_temp("T_air", &v);
        }
        if (sensor_channel_get(sensor, SENSOR_CHAN_HUMIDITY, &v) == 0) {
            print_humidity(&v);
        }
        if (sensor_channel_get(sensor, SENSOR_CHAN_GAS_RES, &v) == 0) {
            printk("Gas=%d ohm  ", v.val1);
        }
        if (sensor_channel_get(sensor, REMOTE_PICO_CHAN_GAS_VALID, &v) == 0
            && v.val1 == 0) {
            printk("(warming) ");
        }

        /* --- MLX90614: object temp --- */
        if (sensor_channel_get(sensor,
                               (enum sensor_channel)REMOTE_PICO_CHAN_OBJECT_TEMP,
                               &v) == 0) {
            print_temp("T_obj", &v);
        }

        /* --- mmWave: occupancy + range --- */
        bool occ = false;
        if (sensor_channel_get(sensor,
                               (enum sensor_channel)REMOTE_PICO_CHAN_OCCUPANCY,
                               &v) == 0) {
            occ = (v.val1 != 0);
            printk("Occupied=%s  ", occ ? "yes" : "no ");
        }
        if (occ && sensor_channel_get(sensor,
                                      (enum sensor_channel)REMOTE_PICO_CHAN_MMWAVE_RANGE,
                                      &v) == 0
            && v.val1 > 0) {
            printk("range=%d cm  ", v.val1);
        }

        /* --- Mic: peak + RMS --- */
        if (sensor_channel_get(sensor,
                               (enum sensor_channel)REMOTE_PICO_CHAN_MIC_PEAK,
                               &v) == 0) {
            printk("MicPk=%d ", v.val1);
        }
        if (sensor_channel_get(sensor,
                               (enum sensor_channel)REMOTE_PICO_CHAN_MIC_RMS,
                               &v) == 0) {
            printk("MicRMS=%d ", v.val1);
        }

        /* --- IRQ source (will stay 0 until thresholds are set) --- */
        if (sensor_channel_get(sensor,
                               (enum sensor_channel)REMOTE_PICO_CHAN_INT_SRC,
                               &v) == 0
            && v.val1 != 0) {
            printk("INT_SRC=0x%02X", v.val1);
        }

        printk("\n");
        k_sleep(POLL_PERIOD);
    }

    return 0;
}