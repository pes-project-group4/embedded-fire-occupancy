#include "bme680.h"

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>

#define I2C_NODE DT_NODELABEL(i2c0)

// calibration registers
#define COEF1_ADDR 0x8A
#define COEF1_LEN 25
#define COEF2_ADDR 0xE1
#define COEF2_LEN 16

// heater registers
#define ADDR_RES_HEAT_VAL 0x00
#define ADDR_RES_HEAT_RANGE 0x02
#define ADDR_RANGE_SW_ERR 0x04

// control values
#define CTRL_HUM_X1 0x01
#define CTRL_MEAS_FORCED ((1u << 5) | (0u << 2) | 0x01)
#define CTRL_GAS_1_RUN_PROF0 ((1u << 4) | 0x00)

// status bits
#define STATUS_NEW_DATA (1u << 7)
#define STATUS_GAS_VALID (1u << 5)
#define STATUS_HEAT_STAB (1u << 4)

#define MEAS_POLL_INTERVAL_MS 10
#define MEAS_POLL_TIMEOUT_MS 500

#define HEATER_AMBIENT_C 25

static struct {
    uint16_t T1;
    int16_t T2;
    int8_t T3;

    uint16_t H1;
    uint16_t H2;
    int8_t H3;
    int8_t H4;
    int8_t H5;
    uint8_t H6;
    int8_t H7;

    int8_t GH1;
    int16_t GH2;
    int8_t GH3;

    uint8_t res_heat_range;
    int8_t res_heat_val;
    int8_t range_sw_err;
} cal;

static const struct device *i2c_dev;
static int32_t t_fine;
static uint32_t meas_wait_ms;
static bool initialized = false;

static int rd8(uint8_t reg, uint8_t *val)
{
    return i2c_reg_read_byte(i2c_dev, BME680_I2C_ADDR, reg, val);
}

static int rdN(uint8_t start_reg, uint8_t *buf, size_t len)
{
    return i2c_burst_read(i2c_dev, BME680_I2C_ADDR, start_reg, buf, len);
}

static int wr8(uint8_t reg, uint8_t val)
{
    return i2c_reg_write_byte(i2c_dev, BME680_I2C_ADDR, reg, val);
}

// calculate temperature
static int32_t compensate_temperature(int32_t adc_T)
{
    int32_t v1;
    int32_t v2;
    int32_t v3;
    int32_t v4;

    v1 = ((adc_T >> 3) - ((int32_t)cal.T1 << 1));
    v2 = (v1 * (int32_t)cal.T2) >> 11;
    v3 = ((v1 >> 1) * (v1 >> 1)) >> 12;
    v4 = (v3 * ((int32_t)cal.T3 << 4)) >> 14;

    t_fine = v2 + v4;

    return (t_fine * 5 + 128) >> 8;
}

// calculate humidity
static uint32_t compensate_humidity(uint16_t adc_H)
{
    int32_t temp_scaled;
    int32_t v1;
    int32_t v2;
    int32_t v3;
    int32_t v4;
    int32_t v5;
    int32_t v6;
    int32_t h;

    temp_scaled = ((t_fine * 5) + 128) >> 8;

    v1 = (int32_t)adc_H - (((int32_t)cal.H1 * 16) +
         (((int32_t)cal.H3 * temp_scaled) / 200));

    v2 = ((int32_t)cal.H2 *
         (((temp_scaled * (int32_t)cal.H4) / 100) +
         (((temp_scaled * ((temp_scaled * (int32_t)cal.H5) / 100)) >> 6) / 100) +
         (1 << 14))) >> 10;

    v3 = v1 * v2;

    v4 = ((int32_t)cal.H6 << 7) + (((int32_t)cal.H7 * temp_scaled) / 100);
    v4 = v4 >> 4;

    v5 = ((v3 >> 14) * (v3 >> 14)) >> 10;
    v6 = (v4 * v5) >> 1;

    h = (((v3 + v6) >> 10) * 1000) >> 12;

    if (h > 100000)
        h = 100000;

    if (h < 0)
        h = 0;

    return (uint32_t)h;
}

static const uint32_t gas_lookup1[16] = {
    2147483647u, 2147483647u, 2147483647u, 2147483647u,
    2147483647u, 2126008810u, 2147483647u, 2130303777u,
    2147483647u, 2147483647u, 2143188679u, 2136746228u,
    2147483647u, 2126008810u, 2147483647u, 2147483647u
};

static const uint32_t gas_lookup2[16] = {
    4096000000u, 2048000000u, 1024000000u, 512000000u,
    255744255u, 127110228u, 64000000u, 32258064u,
    16016016u, 8000000u, 4000000u, 2000000u,
    1000000u, 500000u, 250000u, 125000u
};

// calculate gas resistance
static uint32_t compensate_gas(uint16_t gas_adc, uint8_t gas_range)
{
    int64_t var1;
    int64_t var2;
    int64_t var3;
    uint32_t gas_res;

    var1 = (int64_t)((1340 + (5 * (int64_t)cal.range_sw_err)) *
           (int64_t)gas_lookup1[gas_range]) >> 16;

    var2 = (((int64_t)gas_adc << 15) - (int64_t)16777216) + var1;
    var3 = ((int64_t)gas_lookup2[gas_range] * var1) >> 9;

    gas_res = (uint32_t)((var3 + (var2 >> 1)) / var2);

    return gas_res;
}

// calculate heater setting
static uint8_t calc_res_heat(int16_t target_c, int16_t ambient_c)
{
    int32_t v1;
    int32_t v2;
    int32_t v3;
    int32_t v4;
    int32_t v5;
    int32_t heatr_res_x100;

    if (target_c > 400)
        target_c = 400;

    v1 = (((int32_t)ambient_c * cal.GH3) / 1000) * 256;

    v2 = (cal.GH1 + 784) *
         (((((cal.GH2 + 154009) * target_c * 5) / 100) + 3276800) / 10);

    v3 = v1 + (v2 / 2);
    v4 = v3 / (cal.res_heat_range + 4);
    v5 = (131 * cal.res_heat_val) + 65536;

    heatr_res_x100 = ((v4 / v5) - 250) * 34;

    return (uint8_t)((heatr_res_x100 + 50) / 100);
}

// calculate gas wait value
static uint8_t calc_gas_wait(uint16_t dur_ms)
{
    uint8_t factor = 0;

    if (dur_ms >= 0xFC0)
        return 0xFF;

    while (dur_ms > 0x3F) {
        dur_ms = dur_ms >> 2;
        factor++;
    }

    return (uint8_t)(dur_ms + (factor << 6));
}

static int load_calibration(void)
{
    uint8_t c1[COEF1_LEN];
    uint8_t c2[COEF2_LEN];
    uint8_t extra[5];
    int ret;
    int8_t rse;

    ret = rdN(COEF1_ADDR, c1, COEF1_LEN);
    if (ret < 0)
        return ret;

    ret = rdN(COEF2_ADDR, c2, COEF2_LEN);
    if (ret < 0)
        return ret;

    ret = rdN(0x00, extra, sizeof(extra));
    if (ret < 0)
        return ret;

    cal.T1 = (uint16_t)(c2[8] | (c2[9] << 8));
    cal.T2 = (int16_t)(c1[0] | (c1[1] << 8));
    cal.T3 = (int8_t)c1[2];

    cal.H1 = (uint16_t)(((uint16_t)c2[2] << 4) | (c2[1] & 0x0F));
    cal.H2 = (uint16_t)(((uint16_t)c2[0] << 4) | (c2[1] >> 4));
    cal.H3 = (int8_t)c2[3];
    cal.H4 = (int8_t)c2[4];
    cal.H5 = (int8_t)c2[5];
    cal.H6 = (uint8_t)c2[6];
    cal.H7 = (int8_t)c2[7];

    cal.GH2 = (int16_t)(c2[10] | (c2[11] << 8));
    cal.GH1 = (int8_t)c2[12];
    cal.GH3 = (int8_t)c2[13];

    cal.res_heat_val = (int8_t)extra[0];
    cal.res_heat_range = (extra[2] & 0x30) >> 4;

    // sign extend 4-bit value
    rse = (int8_t)((extra[4] & 0xF0) >> 4);

    if (rse & 0x08)
        rse = rse | 0xF0;

    cal.range_sw_err = rse;

    return 0;
}

int bme680_init(int16_t heater_temp_c, uint16_t heater_duration_ms)
{
    int ret;
    uint8_t chip_id;
    uint8_t res_heat;
    uint8_t gas_wait;

    i2c_dev = DEVICE_DT_GET(I2C_NODE);

    if (!device_is_ready(i2c_dev)) return -ENODEV;

    ret = wr8(BME680_RESET, 0xB6);
    if (ret < 0) return ret;

    k_msleep(10);

    ret = rd8(BME680_ID, &chip_id);
    if (ret < 0) return ret;

    if (chip_id != BME680_CHIP_ID) return -ENODEV;

    ret = load_calibration();
    if (ret < 0) return ret;

    res_heat = calc_res_heat(heater_temp_c, HEATER_AMBIENT_C);
    gas_wait = calc_gas_wait(heater_duration_ms);

    ret = wr8(BME680_RES_HEAT_0, res_heat);
    if (ret < 0) return ret;

    ret = wr8(BME680_GAS_WAIT_0, gas_wait);
    if (ret < 0) return ret;

    ret = wr8(BME680_CTRL_GAS_0, 0x00);
    if (ret < 0) return ret;

    ret = wr8(BME680_CTRL_GAS_1, CTRL_GAS_1_RUN_PROF0);
    if (ret < 0) return ret;

    // humidity must be set before starting measurement
    ret = wr8(BME680_CTRL_HUM, CTRL_HUM_X1);
    if (ret < 0) return ret;

    meas_wait_ms = 20 + heater_duration_ms;
    initialized = true;

    return 0;
}

int bme680_read_all(struct bme680_sample *sample)
{
    int ret;
    uint8_t buf[15];
    uint8_t status;
    uint32_t elapsed;
    int32_t adc_T;
    uint16_t adc_H;
    uint16_t adc_G;
    uint8_t g_rng;

    elapsed = 0;

    if (!initialized || sample == NULL)
        return -EINVAL;

    ret = wr8(BME680_CTRL_MEAS, CTRL_MEAS_FORCED);
    if (ret < 0) return ret;

    // wait until new data is ready
    do {
        k_msleep(MEAS_POLL_INTERVAL_MS);
        elapsed = elapsed + MEAS_POLL_INTERVAL_MS;

        ret = rd8(BME680_MEAS_STATUS_0, &status);
        if (ret < 0) {
            return ret;
        }

        if (status & STATUS_NEW_DATA) {
            break;
        }
    } while (elapsed < (meas_wait_ms + MEAS_POLL_TIMEOUT_MS));

    if (!(status & STATUS_NEW_DATA))
        return -ETIMEDOUT;

    ret = rdN(BME680_MEAS_STATUS_0, buf, sizeof(buf));
    if (ret < 0) return ret;

    // unpack raw sensor data
    adc_T = ((int32_t)buf[5] << 12) |
            ((int32_t)buf[6] << 4) |
            ((int32_t)buf[7] >> 4);

    adc_H = ((uint16_t)buf[8] << 8) | buf[9];

    adc_G = ((uint16_t)buf[13] << 2) | (buf[14] >> 6);
    g_rng = buf[14] & 0x0F;

    sample->temperature_centi_c = compensate_temperature(adc_T);
    sample->humidity_milli_pct = compensate_humidity(adc_H);
    sample->gas_resistance_ohm = compensate_gas(adc_G, g_rng);

    sample->gas_valid = ((status & (STATUS_GAS_VALID | STATUS_HEAT_STAB)) ==
                         (STATUS_GAS_VALID | STATUS_HEAT_STAB));

    return 0;
}