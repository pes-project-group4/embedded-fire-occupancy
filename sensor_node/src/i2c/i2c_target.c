#include "i2c_target.h"
#include "../registers/register_map.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <errno.h>

#define I2C_NODE DT_NODELABEL(i2c1)

static uint8_t reg_ptr;
static bool ptr_loaded; // set on first byte of a write phase

static int on_write_requested(struct i2c_target_config *cfg)
{
    ARG_UNUSED(cfg);

    ptr_loaded = false;
    return 0;
}

static int on_write_received(struct i2c_target_config *cfg, uint8_t val)
{
    ARG_UNUSED(cfg);

    if (!ptr_loaded) {
        reg_ptr = val;
        ptr_loaded = true;
    } else {
        regmap_write(reg_ptr, val);
        reg_ptr++;
    }
    return 0;
}

static int on_read_requested(struct i2c_target_config *cfg, uint8_t *val)
{
    ARG_UNUSED(cfg);

    uint8_t v = 0;
    regmap_read(reg_ptr, &v);
    *val = v;
    reg_ptr++;
    return 0;
}

static int on_read_processed(struct i2c_target_config *cfg, uint8_t *val)
{
    ARG_UNUSED(cfg);

    uint8_t v = 0;
    regmap_read(reg_ptr, &v);
    *val = v;
    reg_ptr++;
    return 0;
}

static int on_stop(struct i2c_target_config *cfg)
{
    ARG_UNUSED(cfg);
    return 0;
}

static const struct i2c_target_callbacks callbacks = {
    .write_requested = on_write_requested,
    .write_received = on_write_received,
    .read_requested = on_read_requested,
    .read_processed = on_read_processed,
    .stop = on_stop,
};

static struct i2c_target_config target_cfg;

int i2c_target_start(uint8_t address)
{
    const struct device *i2c = DEVICE_DT_GET(I2C_NODE);

    if (!device_is_ready(i2c)) {
        printk("[i2c_target] i2c1 not ready\n");
        return -ENODEV;
    }

    target_cfg.address = address;
    target_cfg.flags = 0;
    target_cfg.callbacks = &callbacks;

    int ret = i2c_target_register(i2c, &target_cfg);
    if (ret != 0) {
        printk("[i2c_target] register failed: %d\n", ret);
        return ret;
    }

    printk("[i2c_target] listening on i2c1 @ 0x%02X\n", address);
    return 0;
}
