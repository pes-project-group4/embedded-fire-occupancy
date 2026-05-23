#include "register_map.h"

#include <zephyr/kernel.h>
#include <errno.h>
#include <limits.h>
#include <string.h>

static uint8_t regs[REGMAP_SIZE];
static struct k_spinlock regs_lock;

// check if register can be written
static bool is_writable(uint8_t addr)
{
    switch (addr) {
    case REG_CTRL:
    case REG_INT_SRC:
    case REG_INT_EN:
    case REG_MMW_MAX_GATE:
    case REG_MMW_ABSENCE_0:
    case REG_MMW_ABSENCE_0 + 1:
        return true;

    default:
        if (addr >= REG_MIC_PEAK_TH_0 && addr < REG_MIC_PEAK_TH_0 + 16) {
            return true;
        }

        return false;
    }
}

static void sync_irq_status(void)
{
    if (regs[REG_INT_SRC] != 0) {
        regs[REG_STATUS] |= STATUS_IRQ_PENDING;
    } else {
        regs[REG_STATUS] &= (uint8_t)~STATUS_IRQ_PENDING;
    }
}

// store 16-bit value in little endian
static void store_u16_le(uint8_t addr, uint16_t v)
{
    regs[addr] = (uint8_t)(v & 0xFF);
    regs[addr + 1] = (uint8_t)(v >> 8);
}

// store 32-bit value in little endian
static void store_u32_le(uint8_t addr, uint32_t v)
{
    regs[addr] = (uint8_t)(v & 0xFF);
    regs[addr + 1] = (uint8_t)((v >> 8) & 0xFF);
    regs[addr + 2] = (uint8_t)((v >> 16) & 0xFF);
    regs[addr + 3] = (uint8_t)((v >> 24) & 0xFF);
}

static void store_i32_le(uint8_t addr, int32_t v)
{
    store_u32_le(addr, (uint32_t)v);
}

// read 32-bit signed value
static int32_t load_i32_le(uint8_t addr)
{
    int32_t v;

    v = (int32_t)((uint32_t)regs[addr] |
                  ((uint32_t)regs[addr + 1] << 8) |
                  ((uint32_t)regs[addr + 2] << 16) |
                  ((uint32_t)regs[addr + 3] << 24));

    return v;
}

// read 16-bit unsigned value
static uint16_t load_u16_le(uint8_t addr)
{
    uint16_t v;

    v = (uint16_t)(regs[addr] | ((uint16_t)regs[addr + 1] << 8));

    return v;
}

#define TH_DISABLED INT32_MAX

int regmap_init(void)
{
    memset(regs, 0, sizeof(regs));

    regs[REG_CHIP_ID] = CHIP_ID_VALUE;

    regs[REG_CTRL] = CTRL_EN_BME680 |
                     CTRL_EN_MLX90614 |
                     CTRL_EN_MMWAVE |
                     CTRL_EN_MIC;

    regs[REG_INT_EN] = 0;

    regs[REG_MMW_MAX_GATE] = 5;
    store_u16_le(REG_MMW_ABSENCE_0, 5);

    store_i32_le(REG_MIC_PEAK_TH_0, TH_DISABLED);
    store_i32_le(REG_MIC_RMS_TH_0, TH_DISABLED);
    store_i32_le(REG_T_OBJ_HIGH_0, TH_DISABLED);
    store_i32_le(REG_T_AIR_HIGH_0, TH_DISABLED);

    return 0;
}

int regmap_read(uint8_t addr, uint8_t *out)
{
    if (out == NULL) {
        return -EINVAL;
    }

    K_SPINLOCK(&regs_lock) {
        *out = regs[addr];
    }

    return 0;
}

int regmap_write(uint8_t addr, uint8_t val)
{
    if (!is_writable(addr)) {
        return 0;
    }

    K_SPINLOCK(&regs_lock) {
        regs[addr] = val;
        if (addr == REG_INT_SRC) {
            sync_irq_status();
        }
    }

    return 0;
}

int regmap_read_burst(uint8_t addr, uint8_t *buf, size_t len)
{
    if (buf == NULL) {
        return -EINVAL;
    }

    if ((size_t)addr + len > REGMAP_SIZE) {
        return -EINVAL;
    }

    K_SPINLOCK(&regs_lock) {
        memcpy(buf, &regs[addr], len);
    }

    return 0;
}

int regmap_write_burst(uint8_t addr, const uint8_t *buf, size_t len)
{
    size_t i;
    bool int_src_written = false;

    if (buf == NULL) {
        return -EINVAL;
    }

    if ((size_t)addr + len > REGMAP_SIZE) {
        return -EINVAL;
    }

    K_SPINLOCK(&regs_lock) {
        for (i = 0; i < len; i++) {
            if (is_writable((uint8_t)(addr + i))) {
                regs[addr + i] = buf[i];
                if ((uint8_t)(addr + i) == REG_INT_SRC) {
                    int_src_written = true;
                }
            }
        }

        if (int_src_written) {
            sync_irq_status();
        }
    }

    return 0;
}

// raise interrupt if enabled
static void raise_irq(uint8_t src_bit)
{
    if (regs[REG_INT_EN] & src_bit) {
        regs[REG_INT_SRC] |= src_bit;
        regs[REG_STATUS] |= STATUS_IRQ_PENDING;
    }
}

void regmap_publish_bme680(int32_t temp_centi_c,
                           uint32_t hum_milli_pct,
                           uint32_t gas_ohm,
                           bool gas_valid)
{
    K_SPINLOCK(&regs_lock) {
        int32_t th;

        store_i32_le(REG_BME_TEMP_0, temp_centi_c);
        store_u32_le(REG_BME_HUM_0, hum_milli_pct);
        store_u32_le(REG_BME_GAS_0, gas_ohm);

        if (gas_valid) {
            regs[REG_BME_GAS_VALID] = 1;
        } else {
            regs[REG_BME_GAS_VALID] = 0;
        }

        regs[REG_STATUS] |= STATUS_DATA_READY;

        th = load_i32_le(REG_T_AIR_HIGH_0);

        if (th != TH_DISABLED && temp_centi_c > th) {
            raise_irq(INT_SRC_T_AIR_HIGH);
        }
    }
}

void regmap_publish_mlx90614(int32_t amb_centi_c, int32_t obj_centi_c)
{
    K_SPINLOCK(&regs_lock) {
        int32_t th;

        store_i32_le(REG_MLX_AMB_0, amb_centi_c);
        store_i32_le(REG_MLX_OBJ_0, obj_centi_c);

        regs[REG_STATUS] |= STATUS_DATA_READY;

        th = load_i32_le(REG_T_OBJ_HIGH_0);

        if (th != TH_DISABLED && obj_centi_c > th) {
            raise_irq(INT_SRC_T_OBJ_HIGH);
        }
    }
}

void regmap_publish_mmwave(bool present, uint16_t range_cm)
{
    K_SPINLOCK(&regs_lock) {
        store_u16_le(REG_MMW_RANGE_0, range_cm);

        if (present) {
            regs[REG_MMW_PRESENT] = 1;
        } else {
            regs[REG_MMW_PRESENT] = 0;
        }

        regs[REG_STATUS] |= STATUS_DATA_READY;

        if (present) {
            raise_irq(INT_SRC_MMWAVE);
        }
    }
}

void regmap_publish_mic(int32_t peak, int32_t rms, int32_t baseline)
{
    K_SPINLOCK(&regs_lock) {
        int32_t pth;
        int32_t rth;

        store_i32_le(REG_MIC_PEAK_0, peak);
        store_i32_le(REG_MIC_RMS_0, rms);
        store_i32_le(REG_MIC_BASELINE_0, baseline);

        regs[REG_STATUS] |= STATUS_DATA_READY;

        pth = load_i32_le(REG_MIC_PEAK_TH_0);

        if (pth != TH_DISABLED && peak > pth) {
            raise_irq(INT_SRC_MIC_PEAK);
        }

        rth = load_i32_le(REG_MIC_RMS_TH_0);

        if (rth != TH_DISABLED && rms > rth) {
            raise_irq(INT_SRC_MIC_RMS);
        }
    }
}

uint8_t regmap_get_ctrl(void)
{
    uint8_t v;

    K_SPINLOCK(&regs_lock) {
        v = regs[REG_CTRL];
    }

    return v;
}

void regmap_get_mmwave_config(uint8_t *max_gate, uint16_t *absence_s)
{
    K_SPINLOCK(&regs_lock) {
        if (max_gate) {
            *max_gate = regs[REG_MMW_MAX_GATE];
        }

        if (absence_s) {
            *absence_s = load_u16_le(REG_MMW_ABSENCE_0);
        }
    }
}
