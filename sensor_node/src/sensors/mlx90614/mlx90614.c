#include "mlx90614.h"

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>


#define I2C_NODE DT_NODELABEL(i2c0)

#define MLX90614_RAM_TA 0x06
#define MLX90614_RAM_TOBJ1 0x07
#define MLX90614_I2C_ADDR 0x5A
#define MLX90614_ERROR_FLAG_BIT 0x8000

// raw value is Kelvin * 50 => Celsius * 100 = raw * 2 - 27315
#define KELVIN_OFFSET_CENTI_C 27315


static const struct device *i2c_dev;
static bool initialized = false;


// crc for SMBus PEC
static uint8_t crc8(uint8_t crc, uint8_t data)
{
    crc ^= data;

    for (int i = 0; i < 8; i++) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0x07;
        } else {
            crc <<= 1;
        }
    }

    return crc;
}


// read one word from the sensor and check PEC
static int smbus_read_word(uint8_t cmd, uint16_t *out)
{
    uint8_t rx[3];
    int ret;

    ret = i2c_write_read(i2c_dev, MLX90614_I2C_ADDR, &cmd, 1, rx, 3);
    if (ret < 0) {
        return ret;
    }

    uint8_t pec = 0;

    pec = crc8(pec, (MLX90614_I2C_ADDR << 1) | 0);
    pec = crc8(pec, cmd);
    pec = crc8(pec, (MLX90614_I2C_ADDR << 1) | 1);
    pec = crc8(pec, rx[0]);
    pec = crc8(pec, rx[1]);

    if (pec != rx[2]) {
        return -EIO;
    }

    *out = (uint16_t)rx[0] | ((uint16_t)rx[1] << 8);

    return 0;
}


// convert raw sensor value to centi Celsius
static int raw_to_centi_c(uint16_t raw, int32_t *out)
{
    if (raw & MLX90614_ERROR_FLAG_BIT) {
        return -ERANGE;
    }

    *out = ((int32_t)raw * 2) - KELVIN_OFFSET_CENTI_C;

    return 0;
}


int mlx90614_init(void)
{
    uint16_t raw;
    int ret;

    i2c_dev = DEVICE_DT_GET(I2C_NODE);
    if (!device_is_ready(i2c_dev)) {
        return -ENODEV;
    }

    /* simple test read to check if sensor works */
    ret = smbus_read_word(MLX90614_RAM_TA, &raw);
    if (ret < 0) {
        return ret;
    }

    initialized = true;

    return 0;
}


int mlx90614_read(struct mlx90614_sample *sample)
{
    uint16_t raw_ta;
    uint16_t raw_tobj;
    int ret;

    if (!initialized || sample == NULL) {
        return -EINVAL;
    }

    ret = smbus_read_word(MLX90614_RAM_TA, &raw_ta);
    if (ret < 0) {
        return ret;
    }

    ret = smbus_read_word(MLX90614_RAM_TOBJ1, &raw_tobj);
    if (ret < 0) {
        return ret;
    }

    if (raw_ta & MLX90614_ERROR_FLAG_BIT) {
        return -EIO;
    }

    sample->ambient_centi_c = ((int32_t)raw_ta * 2) - KELVIN_OFFSET_CENTI_C;

    ret = raw_to_centi_c(raw_tobj, &sample->object_centi_c);
    if (ret < 0) {
        return ret;
    }

    return 0;
}
