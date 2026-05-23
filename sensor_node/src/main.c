#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>

#include <stdbool.h>

#include "sensors/bme680/bme680.h"
#include "sensors/mlx90614/mlx90614.h"
#include "sensors/mmwave/mmwave.h"
#include "sensors/spw2430/spw2430.h"
#include "registers/register_map.h"
#include "i2c/i2c_target.h"

/*
 * Sensor node main: threaded design.
 *
 * Each sensor runs in its own thread at its own natural cadence and
 * publishes into the shared register_map. The base station reads from
 * the register map over I2C and applies its own fusion logic.
 *
 * A separate "IRQ pump" thread watches REG_INT_SRC and toggles a
 * GPIO line that the base station's Pico has wired as an edge-triggered
 * interrupt input. When the line trips, the base station reads
 * REG_INT_SRC to find out which threshold caused it.
 *
 * printk() goes to whatever DT has as zephyr,console. With picoprobe on
 * a Pico 2 that's UART0 over the probe's CDC-ACM port.
 */

/* ---------- thread cadences ---------- */
/* Mic gets the fastest cadence because peak detection is transient.
 * BME680 gets the slowest because the gas heater takes ~150 ms and
 * neither air temperature nor humidity changes meaningfully at 1 Hz.
 */
#define BME680_PERIOD_MS    2000
#define MLX_PERIOD_MS       500
#define MMWAVE_PERIOD_MS    500
#define MIC_PERIOD_MS       250
#define IRQ_POLL_MS         20

#define SENSOR_NODE_I2C_ADDR 0x42

/* ---------- thread stacks & priorities ---------- */
/* Lower number = higher priority. Mic > MLX > mmWave > BME680.
 * IRQ pump sits above all of them so a trip propagates without waiting.
 */
#define STACK_SIZE          1536
#define PRIO_IRQ            3
#define PRIO_MIC            4
#define PRIO_MLX            5
#define PRIO_MMWAVE         6
#define PRIO_BME680         7

K_THREAD_STACK_DEFINE(bme680_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(mlx_stack,    STACK_SIZE);
K_THREAD_STACK_DEFINE(mmwave_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(mic_stack,    STACK_SIZE);
K_THREAD_STACK_DEFINE(irq_stack,    512);

static struct k_thread bme680_thread_data;
static struct k_thread mlx_thread_data;
static struct k_thread mmwave_thread_data;
static struct k_thread mic_thread_data;
static struct k_thread irq_thread_data;

/* ---------- I2C bus mutex ---------- */
/* BME680 and MLX90614 share i2c0. Either we trust Zephyr's I2C API to be
 * thread-safe at the transaction level (it usually is) or we wrap the
 * reads ourselves. The mutex is cheap and removes ambiguity.
 */
K_MUTEX_DEFINE(i2c0_mutex);

/* ---------- IRQ output line to base station ---------- */
/*
 * Wire any free GPIO on the sensor-node Pico to a GPIO input on the
 * base-station Pico. Configure base-station side as edge-triggered.
 * The board overlay maps alias `int-out` to GPIO16.
 */
#define INT_OUT_NODE DT_ALIAS(int_out)

static const struct gpio_dt_spec int_out =
    GPIO_DT_SPEC_GET_OR(INT_OUT_NODE, gpios, {0});

static bool int_out_ready = false;

/* ---------- debug print helper ---------- */
/* Prefix every line with uptime and thread name so the picoprobe log is
 * actually readable when four threads are talking at once.
 */
#define DBG(tag, fmt, ...) \
    printk("[%6u ms][%s] " fmt "\n", k_uptime_get_32(), tag, ##__VA_ARGS__)


/* =================================================================== */
/* IRQ pump                                                            */
/* =================================================================== */
/*
 * Watch REG_INT_SRC and drive the interrupt line.
 *
 * Pulse semantics: rising edge means "something tripped, come look at
 * REG_INT_SRC". The base station I2C-reads INT_SRC, processes it, and
 * writes 0 back to clear the bits. Until INT_SRC is cleared, the line
 * stays high.
 */
static void irq_pump_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    uint8_t last_src = 0;

    while (1) {
        uint8_t status;
        uint8_t src;

        regmap_read(REG_STATUS, &status);
        regmap_read(REG_INT_SRC, &src);

        if (int_out_ready) {
            int want = (src != 0) ? 1 : 0;
            gpio_pin_set_dt(&int_out, want);
        }

        if (src != last_src) {
            DBG("irq", "INT_SRC=0x%02X status=0x%02X "
                "(mic_pk=%d mic_rms=%d obj=%d air=%d mmw=%d)",
                src, status,
                !!(src & INT_SRC_MIC_PEAK),
                !!(src & INT_SRC_MIC_RMS),
                !!(src & INT_SRC_T_OBJ_HIGH),
                !!(src & INT_SRC_T_AIR_HIGH),
                !!(src & INT_SRC_MMWAVE));
            last_src = src;
        }

        k_msleep(IRQ_POLL_MS);
    }
}

static void irq_init(void)
{
    if (!device_is_ready(int_out.port)) {
        DBG("irq", "INT_OUT pin not available (skipping)");
        int_out_ready = false;
        return;
    }

    int ret = gpio_pin_configure_dt(&int_out, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        DBG("irq", "configure failed: %d", ret);
        int_out_ready = false;
        return;
    }

    int_out_ready = true;
    DBG("irq", "INT_OUT ready");
}


/* =================================================================== */
/* BME680 thread                                                       */
/* =================================================================== */
static void bme680_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    /* Warm up with a discarded read so the gas resistance settles
     * before the first published sample. */
    struct bme680_sample s;

    k_mutex_lock(&i2c0_mutex, K_FOREVER);
    (void)bme680_read_all(&s);
    k_mutex_unlock(&i2c0_mutex);

    while (1) {
        uint8_t ctrl = regmap_get_ctrl();
        if (!(ctrl & CTRL_EN_BME680)) {
            k_msleep(BME680_PERIOD_MS);
            continue;
        }

        k_mutex_lock(&i2c0_mutex, K_FOREVER);
        int ret = bme680_read_all(&s);
        k_mutex_unlock(&i2c0_mutex);

        if (ret == 0) {
            regmap_publish_bme680(s.temperature_centi_c,
                                  s.humidity_milli_pct,
                                  s.gas_resistance_ohm,
                                  s.gas_valid);

            int t_whole = s.temperature_centi_c / 100;
            int t_frac  = s.temperature_centi_c % 100;
            if (t_frac < 0) t_frac = -t_frac;

            DBG("bme680",
                "T_air=%d.%02d C  RH=%u.%03u %%  Gas=%u ohm%s",
                t_whole, t_frac,
                (unsigned)(s.humidity_milli_pct / 1000),
                (unsigned)(s.humidity_milli_pct % 1000),
                (unsigned)s.gas_resistance_ohm,
                s.gas_valid ? "" : " (warming)");
        } else {
            DBG("bme680", "read failed: %d", ret);
        }

        k_msleep(BME680_PERIOD_MS);
    }
}


/* =================================================================== */
/* MLX90614 thread                                                     */
/* =================================================================== */
static void mlx_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    while (1) {
        uint8_t ctrl = regmap_get_ctrl();
        if (!(ctrl & CTRL_EN_MLX90614)) {
            k_msleep(MLX_PERIOD_MS);
            continue;
        }

        struct mlx90614_sample s;

        k_mutex_lock(&i2c0_mutex, K_FOREVER);
        int ret = mlx90614_read(&s);
        k_mutex_unlock(&i2c0_mutex);

        if (ret == 0) {
            regmap_publish_mlx90614(s.ambient_centi_c,
                                    s.object_centi_c);

            int ow = s.object_centi_c / 100;
            int of = s.object_centi_c % 100;
            if (of < 0) of = -of;
            int aw = s.ambient_centi_c / 100;
            int af = s.ambient_centi_c % 100;
            if (af < 0) af = -af;

            DBG("mlx", "T_obj=%d.%02d C  T_amb=%d.%02d C",
                ow, of, aw, af);
        } else {
            DBG("mlx", "read failed: %d", ret);
        }

        k_msleep(MLX_PERIOD_MS);
    }
}


/* =================================================================== */
/* mmWave thread                                                       */
/* =================================================================== */
/* The driver already maintains state via its own UART ISR. This thread
 * just snapshots and republishes. We treat "occupied" as: present AND
 * not stale (>2 s without an update probably means UART is silent).
 */
static void mmwave_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    bool last_occ = false;
    uint16_t last_range = 0;
    uint8_t last_gate = 0xFF;
    uint16_t last_absence = 0xFFFF;

    while (1) {
        uint8_t ctrl = regmap_get_ctrl();
        if (!(ctrl & CTRL_EN_MMWAVE)) {
            k_msleep(MMWAVE_PERIOD_MS);
            continue;
        }

        uint8_t gate;
        uint16_t absence;
        regmap_get_mmwave_config(&gate, &absence);

        if (gate != last_gate || absence != last_absence) {
            struct mmwave_config cfg = {
                .max_range_gate = gate,
                .absence_delay_s = absence,
            };

            int ret = mmwave_apply_config(&cfg);
            if (ret == 0) {
                DBG("mmwave", "config: max_gate=%u (~%u cm), absence=%us",
                    cfg.max_range_gate,
                    (unsigned)(cfg.max_range_gate + 1) * 70,
                    cfg.absence_delay_s);
                last_gate = gate;
                last_absence = absence;
            } else {
                DBG("mmwave", "config apply failed: %d", ret);
            }
        }

        struct mmwave_state st;
        if (mmwave_get_state(&st) == 0) {
            bool occ = mmwave_is_occupied(2000);
            regmap_publish_mmwave(occ, st.range_cm);

            /* Only log on change to keep the console legible. */
            if (occ != last_occ || st.range_cm != last_range) {
                DBG("mmwave", "occupied=%s range=%u cm",
                    occ ? "yes" : "no",
                    (unsigned)st.range_cm);
                last_occ   = occ;
                last_range = st.range_cm;
            }
        }

        k_msleep(MMWAVE_PERIOD_MS);
    }
}


/* =================================================================== */
/* Microphone thread                                                   */
/* =================================================================== */
static void mic_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    int32_t peak = 0;
    int32_t rms  = 0;

    /* Throttle the chatty per-window prints to once per second so the
     * other threads' lines are still visible. */
    uint32_t last_print = 0;

    while (1) {
        uint8_t ctrl = regmap_get_ctrl();
        if (!(ctrl & CTRL_EN_MIC)) {
            k_msleep(MIC_PERIOD_MS);
            continue;
        }

        int ret = spw2430_read_level(SPW2430_DEFAULT_WINDOW_SAMPLES,
                                     &peak, &rms);
        if (ret == 0) {
            regmap_publish_mic(peak, rms, spw2430_get_baseline());

            uint32_t now = k_uptime_get_32();
            if (now - last_print >= 1000) {
                DBG("mic", "peak=%d rms=%d baseline=%d",
                    (int)peak, (int)rms,
                    (int)spw2430_get_baseline());
                last_print = now;
            }
        } else {
            DBG("mic", "read failed: %d", ret);
        }

        k_msleep(MIC_PERIOD_MS);
    }
}


/* =================================================================== */
/* main                                                                */
/* =================================================================== */
int main(void)
{
    int ret;
    uint8_t enabled;
    bool bme680_ready = false;
    bool mlx_ready = false;
    bool mmwave_ready = false;
    bool mic_ready = false;

    printk("\n=== Sensor node starting (threaded) ===\n");

    regmap_init();
    enabled = regmap_get_ctrl();
    DBG("main", "register map ready (chip id 0x%02X)", CHIP_ID_VALUE);

    /* --- BME680 -------------------------------------------------- */
    ret = bme680_init(320, 150);
    if (ret < 0) {
        DBG("main", "bme680_init failed: %d", ret);
        enabled &= (uint8_t)~CTRL_EN_BME680;
    } else {
        bme680_ready = true;
        DBG("main", "BME680 ready");
    }

    /* --- MLX90614 ------------------------------------------------ */
    ret = mlx90614_init();
    if (ret < 0) {
        DBG("main", "mlx90614_init failed: %d", ret);
        enabled &= (uint8_t)~CTRL_EN_MLX90614;
    } else {
        mlx_ready = true;
        DBG("main", "MLX90614 ready");
    }

    /* --- HMMD mmWave --------------------------------------------- */
    ret = mmwave_init();
    if (ret < 0) {
        DBG("main", "mmwave_init failed: %d", ret);
        enabled &= (uint8_t)~CTRL_EN_MMWAVE;
    } else {
        mmwave_ready = true;
        DBG("main", "HMMD mmWave ready");
    }

    /* --- SPW2430 ------------------------------------------------- */
    ret = spw2430_init();
    if (ret < 0) {
        DBG("main", "spw2430_init failed: %d", ret);
        enabled &= (uint8_t)~CTRL_EN_MIC;
    } else {
        DBG("main", "SPW2430 ready, calibrating (%d samples)...",
            SPW2430_DEFAULT_CALIB_SAMPLES);

        ret = spw2430_calibrate(SPW2430_DEFAULT_CALIB_SAMPLES);
        if (ret < 0) {
            DBG("main", "spw2430 calibrate failed: %d", ret);
            enabled &= (uint8_t)~CTRL_EN_MIC;
        } else {
            mic_ready = true;
            DBG("main", "mic baseline = %d",
                (int)spw2430_get_baseline());
        }
    }

    regmap_write(REG_CTRL, enabled);

    ret = i2c_target_start(SENSOR_NODE_I2C_ADDR);
    if (ret < 0) {
        DBG("main", "i2c_target_start failed: %d", ret);
    }

    /* --- IRQ output ---------------------------------------------- */
    irq_init();

    /* --- launch threads ------------------------------------------ */
    if (bme680_ready) {
        k_thread_create(&bme680_thread_data, bme680_stack, STACK_SIZE,
                        bme680_entry, NULL, NULL, NULL,
                        PRIO_BME680, 0, K_NO_WAIT);
        k_thread_name_set(&bme680_thread_data, "bme680");
    }

    if (mlx_ready) {
        k_thread_create(&mlx_thread_data, mlx_stack, STACK_SIZE,
                        mlx_entry, NULL, NULL, NULL,
                        PRIO_MLX, 0, K_NO_WAIT);
        k_thread_name_set(&mlx_thread_data, "mlx90614");
    }

    if (mmwave_ready) {
        k_thread_create(&mmwave_thread_data, mmwave_stack, STACK_SIZE,
                        mmwave_entry, NULL, NULL, NULL,
                        PRIO_MMWAVE, 0, K_NO_WAIT);
        k_thread_name_set(&mmwave_thread_data, "mmwave");
    }

    if (mic_ready) {
        k_thread_create(&mic_thread_data, mic_stack, STACK_SIZE,
                        mic_entry, NULL, NULL, NULL,
                        PRIO_MIC, 0, K_NO_WAIT);
        k_thread_name_set(&mic_thread_data, "mic");
    }

    k_thread_create(&irq_thread_data, irq_stack, 512,
                    irq_pump_entry, NULL, NULL, NULL,
                    PRIO_IRQ, 0, K_NO_WAIT);
    k_thread_name_set(&irq_thread_data, "irq");

    DBG("main", "all threads launched");

    /* Main is done; the threads carry on. Sleep forever so the
     * thread doesn't return (which would cause a fault on some
     * configurations). */
    while (1) {
        k_sleep(K_FOREVER);
    }

    return 0;
}
