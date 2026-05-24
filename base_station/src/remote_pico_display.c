#include "remote_pico_display.h"

#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>

#include <stdbool.h>

#include "../drivers/sensor/remote_pico/remote_pico.h"

static void print_temp(const char *label, const struct sensor_value *v)
{
    /* sensor_value::val2 is signed microunits; preserve sign cleanly. */
    int frac = v->val2 / 10000;

    if (frac < 0) {
        frac = -frac;
    }

    printk("%s=%d.%02d C  ", label, v->val1, frac);
}

static void print_humidity(const struct sensor_value *v)
{
    int frac = v->val2 / 1000;

    if (frac < 0) {
        frac = -frac;
    }

    printk("RH=%d.%03d %%  ", v->val1, frac);
}

void remote_pico_print_irq_sources(uint8_t src)
{
    printk("INT_SRC=0x%02X", src);

    if (src & REMOTE_PICO_INT_T_OBJ_HIGH) {
        printk(" T_OBJ_HIGH");
    }
    if (src & REMOTE_PICO_INT_T_AIR_HIGH) {
        printk(" T_AIR_HIGH");
    }
    if (src & REMOTE_PICO_INT_MIC_PEAK) {
        printk(" MIC_PEAK");
    }
    if (src & REMOTE_PICO_INT_MIC_RMS) {
        printk(" MIC_RMS");
    }
    if (src & REMOTE_PICO_INT_MMWAVE) {
        printk(" MMWAVE");
    }
}

void remote_pico_print_sensor_snapshot(const struct device *sensor)
{
    struct sensor_value v;

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

    if (sensor_channel_get(sensor,
                           (enum sensor_channel)REMOTE_PICO_CHAN_OBJECT_TEMP,
                           &v) == 0) {
        print_temp("T_obj", &v);
    }

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
}
