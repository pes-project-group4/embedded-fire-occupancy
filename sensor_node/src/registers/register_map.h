#ifndef _REGISTER_MAP_H_
#define _REGISTER_MAP_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// control and status registers
#define REG_STATUS 0x00
#define REG_CTRL 0x01
#define REG_INT_SRC 0x02
#define REG_INT_EN 0x03

// sensor enable bits
#define CTRL_EN_BME680 (1u << 0)
#define CTRL_EN_MLX90614 (1u << 1)
#define CTRL_EN_MMWAVE (1u << 2)
#define CTRL_EN_MIC (1u << 3)

// interrupt source bits
#define INT_SRC_T_OBJ_HIGH (1u << 2)

// status bits
#define STATUS_DATA_READY (1u << 0)
#define STATUS_IRQ_PENDING (1u << 1)

// BME680 registers
#define REG_BME_TEMP_0 0x10
#define REG_BME_HUM_0 0x14
#define REG_BME_GAS_0 0x18
#define REG_BME_GAS_VALID 0x1C

// MLX90614 registers
#define REG_MLX_AMB_0 0x20
#define REG_MLX_OBJ_0 0x24

// mmWave registers
#define REG_MMW_RANGE_0 0x30
#define REG_MMW_PRESENT 0x32
#define REG_MMW_MAX_GATE 0x33
#define REG_MMW_ABSENCE_0 0x34

// microphone registers
#define REG_MIC_PEAK_0 0x40
#define REG_MIC_RMS_0 0x44
#define REG_MIC_BASELINE_0 0x48

// threshold registers
#define REG_T_OBJ_HIGH_0 0x68

// chip ID
#define REG_CHIP_ID 0xFF
#define CHIP_ID_VALUE 0x42

#define REGMAP_SIZE 0x100

int regmap_init(void);

int regmap_read(uint8_t addr, uint8_t *out);
int regmap_write(uint8_t addr, uint8_t val);

int regmap_read_burst(uint8_t addr, uint8_t *buf, size_t len);
int regmap_write_burst(uint8_t addr, const uint8_t *buf, size_t len);

void regmap_publish_bme680(int32_t temp_centi_c, uint32_t hum_milli_pct, uint32_t gas_ohm, bool gas_valid);
void regmap_publish_mlx90614(int32_t amb_centi_c, int32_t obj_centi_c);
void regmap_publish_mmwave(bool present, uint16_t range_cm);
void regmap_publish_mic(int32_t peak, int32_t rms, int32_t baseline);

uint8_t regmap_get_ctrl(void);
void regmap_get_mmwave_config(uint8_t *max_gate, uint16_t *absence_s);

#endif
