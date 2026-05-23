#include "i2c_target.h"
#include "../registers/register_map.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <errno.h>

/*
 * I2C target glue using Zephyr's standard i2c_target_register API.
 *
 * Wire protocol (matches the base-station driver's expectations):
 *
 *   Write transaction:
 *       [ADDR_W] [REG] [VAL0] [VAL1] ...
 *     The first byte after the address is the register pointer.
 *     Each subsequent byte is written to (pointer)++ via regmap_write().
 *
 *   Combined read (write-then-read with repeated start):
 *       [ADDR_W] [REG]  Sr  [ADDR_R] [VAL0] [VAL1] ...
 *     The pointer is loaded from the write phase, then bytes flow back
 *     from (pointer)++ via regmap_read().
 *
 *   Bare read (reuses the previously-stored pointer):
 *       [ADDR_R] [VAL0] [VAL1] ...
 *
 * The DesignWare I2C controller in the RP2040/RP2350 already implements
 * target mode in Zephyr's i2c_dw driver, so this is all we need.
 */

#define I2C_NODE DT_NODELABEL(i2c1)

static uint8_t  reg_ptr;
static bool     ptr_loaded;       /* set on first byte of a write phase */

static int on_write_requested(struct i2c_target_config *cfg)
{
    ARG_UNUSED(cfg);
    /* Start of a write transaction. The next byte is the register
     * pointer; subsequent bytes are data. */
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
    /* Read transaction. Either the controller just did a repeated start
     * after writing the pointer, or it's reusing the previously-stored
     * pointer for a chained read. Either way, return regs[reg_ptr] and
     * advance. */
    uint8_t v = 0;
    regmap_read(reg_ptr, &v);
    *val = v;
    reg_ptr++;
    return 0;
}

static int on_read_processed(struct i2c_target_config *cfg, uint8_t *val)
{
    ARG_UNUSED(cfg);
    /* Controller ACK'd the previous byte and wants another. */
    uint8_t v = 0;
    regmap_read(reg_ptr, &v);
    *val = v;
    reg_ptr++;
    return 0;
}

static int on_stop(struct i2c_target_config *cfg)
{
    ARG_UNUSED(cfg);
    /* Do NOT reset reg_ptr here. Bare-read transactions rely on the
     * pointer being preserved across STOPs. The pointer is only
     * reset (logically, via ptr_loaded) at the start of a write phase. */
    return 0;
}

static const struct i2c_target_callbacks callbacks = {
    .write_requested = on_write_requested,
    .write_received  = on_write_received,
    .read_requested  = on_read_requested,
    .read_processed  = on_read_processed,
    .stop            = on_stop,
};

static struct i2c_target_config target_cfg;

int i2c_target_start(uint8_t address)
{
    const struct device *i2c = DEVICE_DT_GET(I2C_NODE);

    if (!device_is_ready(i2c)) {
        printk("[i2c_target] i2c1 not ready\n");
        return -ENODEV;
    }

    target_cfg.address   = address;
    target_cfg.flags     = 0;
    target_cfg.callbacks = &callbacks;

    int ret = i2c_target_register(i2c, &target_cfg);
    if (ret != 0) {
        printk("[i2c_target] register failed: %d\n", ret);
        return ret;
    }

    printk("[i2c_target] listening on i2c1 @ 0x%02X\n", address);
    return 0;
}
