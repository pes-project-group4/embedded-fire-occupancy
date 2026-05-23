#define DT_DRV_COMPAT vnd_remote_pico

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include <errno.h>
#include <string.h>

LOG_MODULE_REGISTER(remote_pico, CONFIG_SENSOR_LOG_LEVEL);

/*
 * Layout of the sensor node's register map (mirrors register_map.h on the
 * sensor-node side). We don't include that header directly because it
 * pulls in Zephyr headers from a different build, so we re-declare the
 * offsets here. Keep these in sync with register_map.h.
 */

/* control / status */
#define REG_STATUS          0x00
#define REG_CTRL            0x01
#define REG_INT_SRC         0x02
#define REG_INT_EN          0x03

/* BME680 */
#define REG_BME_TEMP_0      0x10  /* int32, centi-C */
#define REG_BME_HUM_0       0x14  /* uint32, milli-% */
#define REG_BME_GAS_0       0x18  /* uint32, ohm */
#define REG_BME_GAS_VALID   0x1C  /* uint8 */

/* MLX90614 */
#define REG_MLX_AMB_0       0x20  /* int32, centi-C */
#define REG_MLX_OBJ_0       0x24  /* int32, centi-C */

/* mmWave */
#define REG_MMW_RANGE_0     0x30  /* uint16, cm */
#define REG_MMW_PRESENT     0x32  /* uint8 */

/* Mic */
#define REG_MIC_PEAK_0      0x40  /* int32, ADC counts */
#define REG_MIC_RMS_0       0x44  /* int32, ADC counts */
#define REG_MIC_BASELINE_0  0x48  /* int32, ADC counts */

#define REG_CHIP_ID         0xFF
#define CHIP_ID_EXPECTED    0x42

/* Burst window: start at REG_STATUS, read through end of mic baseline.
 * That's 0x00..0x4B = 76 bytes. The Pico's I2C target buffer is usually
 * larger than this; if it's a problem on the target side we can split
 * into two reads.
 */
#define BURST_START         0x00
#define BURST_LEN           0x4C  /* 76 bytes */

/*
 * Custom (private) sensor channels for the values that don't have a
 * standard Zephyr counterpart. Numbering starts at SENSOR_CHAN_PRIV_START.
 */
enum remote_pico_chan {
    REMOTE_PICO_CHAN_OBJECT_TEMP = SENSOR_CHAN_PRIV_START,
    REMOTE_PICO_CHAN_MMWAVE_RANGE,
    REMOTE_PICO_CHAN_OCCUPANCY,
    REMOTE_PICO_CHAN_MIC_PEAK,
    REMOTE_PICO_CHAN_MIC_RMS,
    REMOTE_PICO_CHAN_GAS_VALID,
    REMOTE_PICO_CHAN_INT_SRC,
};

struct remote_pico_config {
    struct i2c_dt_spec bus;
};

struct remote_pico_data {
    uint8_t buf[BURST_LEN];
};

/* --- Helpers to decode little-endian fields from the burst buffer --- */
static inline int32_t buf_i32(const uint8_t *p)
{
    return (int32_t)sys_get_le32(p);
}
static inline uint32_t buf_u32(const uint8_t *p)
{
    return sys_get_le32(p);
}
static inline uint16_t buf_u16(const uint8_t *p)
{
    return sys_get_le16(p);
}

/* Convert centi-Celsius to sensor_value (val1 = integer part,
 * val2 = micro fractional). */
static void centi_c_to_sv(int32_t centi_c, struct sensor_value *v)
{
    v->val1 = centi_c / 100;
    /* sensor_value::val2 is micro-units (1e-6). centi-degree
     * remainder * 10000 gives the right scale. */
    v->val2 = (centi_c % 100) * 10000;
}

/* Convert milli-percent to sensor_value. */
static void milli_pct_to_sv(uint32_t milli_pct, struct sensor_value *v)
{
    v->val1 = (int32_t)(milli_pct / 1000);
    /* (milli_pct % 1000) is a millipoint; * 1000 → micro. */
    v->val2 = (int32_t)((milli_pct % 1000) * 1000);
}


/* =================================================================== */
/* sample_fetch                                                        */
/* =================================================================== */
/*
 * One I2C burst read populates the entire local cache. We do it
 * unconditionally regardless of `chan` — there's no per-sensor fetch
 * since the cost dominator is the I2C round-trip start/stop, not the
 * byte count.
 */
static int remote_pico_sample_fetch(const struct device *dev,
                                    enum sensor_channel chan)
{
    const struct remote_pico_config *cfg = dev->config;
    struct remote_pico_data *data = dev->data;

    ARG_UNUSED(chan);

    uint8_t start = BURST_START;
    int ret = i2c_write_read_dt(&cfg->bus, &start, 1,
                                data->buf, sizeof(data->buf));
    if (ret != 0) {
        LOG_WRN("burst read failed: %d", ret);
        return ret;
    }

    LOG_DBG("fetched %u bytes; status=0x%02X int_src=0x%02X",
            (unsigned)sizeof(data->buf),
            data->buf[REG_STATUS],
            data->buf[REG_INT_SRC]);

    return 0;
}


/* =================================================================== */
/* channel_get                                                         */
/* =================================================================== */
static int remote_pico_channel_get(const struct device *dev,
                                   enum sensor_channel chan,
                                   struct sensor_value *val)
{
    struct remote_pico_data *data = dev->data;
    const uint8_t *b = data->buf;

    switch ((int)chan) {

    case SENSOR_CHAN_AMBIENT_TEMP: {
        /* Air temperature from BME680. */
        centi_c_to_sv(buf_i32(&b[REG_BME_TEMP_0]), val);
        return 0;
    }

    case SENSOR_CHAN_HUMIDITY: {
        milli_pct_to_sv(buf_u32(&b[REG_BME_HUM_0]), val);
        return 0;
    }

    case SENSOR_CHAN_GAS_RES: {
        /* Gas resistance in ohms — whole-number, no fractional part. */
        val->val1 = (int32_t)buf_u32(&b[REG_BME_GAS_0]);
        val->val2 = 0;
        return 0;
    }

    case REMOTE_PICO_CHAN_OBJECT_TEMP: {
        /* Object temperature from MLX90614 IR. */
        centi_c_to_sv(buf_i32(&b[REG_MLX_OBJ_0]), val);
        return 0;
    }

    case REMOTE_PICO_CHAN_MMWAVE_RANGE: {
        /* Range in cm. */
        val->val1 = buf_u16(&b[REG_MMW_RANGE_0]);
        val->val2 = 0;
        return 0;
    }

    case REMOTE_PICO_CHAN_OCCUPANCY: {
        val->val1 = b[REG_MMW_PRESENT] ? 1 : 0;
        val->val2 = 0;
        return 0;
    }

    case REMOTE_PICO_CHAN_MIC_PEAK: {
        val->val1 = buf_i32(&b[REG_MIC_PEAK_0]);
        val->val2 = 0;
        return 0;
    }

    case REMOTE_PICO_CHAN_MIC_RMS: {
        val->val1 = buf_i32(&b[REG_MIC_RMS_0]);
        val->val2 = 0;
        return 0;
    }

    case REMOTE_PICO_CHAN_GAS_VALID: {
        val->val1 = b[REG_BME_GAS_VALID] ? 1 : 0;
        val->val2 = 0;
        return 0;
    }

    case REMOTE_PICO_CHAN_INT_SRC: {
        val->val1 = b[REG_INT_SRC];
        val->val2 = 0;
        return 0;
    }

    default:
        return -ENOTSUP;
    }
}


/* =================================================================== */
/* init                                                                */
/* =================================================================== */
/*
 * Verify the I2C bus is up and that the sensor node answers with the
 * expected chip ID. If the chip ID is wrong, we still return 0 (the
 * device exists, just isn't speaking our protocol yet) — the user can
 * see this in the log. Returning -ENODEV would make device_is_ready()
 * fail for the consumer and the whole app would refuse to start, which
 * is unfriendly when the sensor node is just booting up.
 */
static int remote_pico_init(const struct device *dev)
{
    const struct remote_pico_config *cfg = dev->config;

    if (!i2c_is_ready_dt(&cfg->bus)) {
        LOG_ERR("I2C bus %s not ready", cfg->bus.bus->name);
        return -ENODEV;
    }

    uint8_t addr = REG_CHIP_ID;
    uint8_t id = 0;

    int ret = i2c_write_read_dt(&cfg->bus, &addr, 1, &id, 1);
    if (ret != 0) {
        LOG_WRN("chip ID read failed: %d (sensor node off?)", ret);
        /* Still succeed; periodic fetch will retry. */
        return 0;
    }

    if (id != CHIP_ID_EXPECTED) {
        LOG_WRN("unexpected chip ID 0x%02X (expected 0x%02X)",
                id, CHIP_ID_EXPECTED);
    } else {
        LOG_INF("remote pico ready, chip ID 0x%02X", id);
    }

    return 0;
}


static DEVICE_API(sensor, remote_pico_api) = {
    .sample_fetch = remote_pico_sample_fetch,
    .channel_get  = remote_pico_channel_get,
};

#define REMOTE_PICO_DEFINE(inst)                                          \
    static struct remote_pico_data remote_pico_data_##inst;               \
                                                                          \
    static const struct remote_pico_config remote_pico_config_##inst = {  \
        .bus = I2C_DT_SPEC_INST_GET(inst),                                \
    };                                                                    \
                                                                          \
    SENSOR_DEVICE_DT_INST_DEFINE(inst,                                    \
                                 remote_pico_init,                        \
                                 NULL,                                    \
                                 &remote_pico_data_##inst,                \
                                 &remote_pico_config_##inst,              \
                                 POST_KERNEL,                             \
                                 CONFIG_SENSOR_INIT_PRIORITY,             \
                                 &remote_pico_api);

DT_INST_FOREACH_STATUS_OKAY(REMOTE_PICO_DEFINE)
