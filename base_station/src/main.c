#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>

#include <errno.h>
#include <stdbool.h>

#include "../drivers/sensor/remote_pico/remote_pico.h"

/*
 * Base station application.
 *
 * Polls the sensor node every second through the Zephyr Sensor API.
 * One sample_fetch() triggers an I2C burst read on the driver side;
 * subsequent channel_get() calls just decode from the cached buffer,
 * so they're cheap.
 *
 * GPIO16 is also wired to the sensor node's GPIO16 interrupt output.
 * The demo configures the sensor node to raise that line when MLX90614
 * object temperature crosses OBJECT_TEMP_IRQ_CENTI_C.
 *
 * Output goes to UART0 via printk (picoprobe CDC-ACM on Pico 2).
 */

#define REMOTE_PICO_NODE DT_NODELABEL(remote_pico)
#define POLL_PERIOD K_SECONDS(1)
#define IRQ_CONFIG_RETRY_MS 2000
#define OBJECT_TEMP_IRQ_CENTI_C 3000

static const struct gpio_dt_spec remote_irq =
    GPIO_DT_SPEC_GET_OR(REMOTE_PICO_NODE, int_gpios, {0});

static const struct device *remote_sensor;
static struct gpio_callback remote_irq_cb;
static K_MUTEX_DEFINE(remote_sensor_lock);
static bool interrupt_demo_configured;
static uint32_t last_irq_config_try_ms;

static void remote_irq_work_handler(struct k_work *work);
static K_WORK_DEFINE(remote_irq_work, remote_irq_work_handler);

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

static void print_irq_sources(uint8_t src)
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

static void remote_irq_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (remote_sensor == NULL) {
        return;
    }

    k_mutex_lock(&remote_sensor_lock, K_FOREVER);

    int ret = sensor_sample_fetch(remote_sensor);
    if (ret != 0) {
        printk("[%6u ms][irq] sample_fetch failed: %d\n",
               k_uptime_get_32(), ret);
        k_mutex_unlock(&remote_sensor_lock);
        return;
    }

    struct sensor_value v;
    ret = sensor_channel_get(remote_sensor,
                             (enum sensor_channel)REMOTE_PICO_CHAN_INT_SRC,
                             &v);
    if (ret == 0 && v.val1 != 0) {
        printk("[%6u ms][irq] ", k_uptime_get_32());
        print_irq_sources((uint8_t)v.val1);
        printk("\n");

        ret = remote_pico_clear_interrupts(remote_sensor);
        if (ret != 0) {
            printk("[%6u ms][irq] clear failed: %d\n",
                   k_uptime_get_32(), ret);
        }
    }

    k_mutex_unlock(&remote_sensor_lock);
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

static int configure_remote_interrupt_demo(const struct device *sensor)
{
    int ret;

    ret = remote_pico_set_interrupts(sensor, 0);
    if (ret != 0) {
        printk("interrupt demo: disable INT_EN failed: %d\n", ret);
        return ret;
    }

    ret = remote_pico_set_object_temp_high(sensor, OBJECT_TEMP_IRQ_CENTI_C);
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
           OBJECT_TEMP_IRQ_CENTI_C / 100,
           OBJECT_TEMP_IRQ_CENTI_C % 100);

    return 0;
}

static void try_configure_remote_interrupt_demo(const struct device *sensor,
                                                bool force)
{
    uint32_t now = k_uptime_get_32();

    if (interrupt_demo_configured) {
        return;
    }

    if (!force && (now - last_irq_config_try_ms) < IRQ_CONFIG_RETRY_MS) {
        return;
    }

    last_irq_config_try_ms = now;

    int ret = configure_remote_interrupt_demo(sensor);
    if (ret == 0) {
        interrupt_demo_configured = true;
        return;
    }

    printk("interrupt demo config pending: %d "
           "(check sensor-node firmware, power, I2C wiring)\n", ret);
}

int main(void)
{
    const struct device *sensor = DEVICE_DT_GET(DT_NODELABEL(remote_pico));

    printk("\n=== Base station starting ===\n");

    if (!device_is_ready(sensor)) {
        printk("remote_pico device not ready\n");
        return 0;
    }

    remote_sensor = sensor;

    int ret = configure_irq_gpio();
    if (ret != 0) {
        printk("continuing with polling only\n");
    }

    try_configure_remote_interrupt_demo(sensor, true);

    printk("remote_pico ready, polling every 1 s\n\n");

    while (1) {
        struct sensor_value v;
        bool clear_irq = false;

        k_mutex_lock(&remote_sensor_lock, K_FOREVER);
        ret = sensor_sample_fetch(sensor);
        if (ret != 0) {
            printk("[%6u ms] sample_fetch failed: %d\n",
                   k_uptime_get_32(), ret);
            k_mutex_unlock(&remote_sensor_lock);
            k_sleep(POLL_PERIOD);
            continue;
        }

        try_configure_remote_interrupt_demo(sensor, false);

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

        /* --- IRQ source, also handled by the GPIO callback path --- */
        if (sensor_channel_get(sensor,
                               (enum sensor_channel)REMOTE_PICO_CHAN_INT_SRC,
                               &v) == 0
            && v.val1 != 0) {
            print_irq_sources((uint8_t)v.val1);
            clear_irq = true;
        }

        printk("\n");

        if (clear_irq) {
            ret = remote_pico_clear_interrupts(sensor);
            if (ret != 0) {
                printk("[%6u ms] interrupt clear failed: %d\n",
                       k_uptime_get_32(), ret);
            }
        }

        k_mutex_unlock(&remote_sensor_lock);
        k_sleep(POLL_PERIOD);
    }

    return 0;
}
