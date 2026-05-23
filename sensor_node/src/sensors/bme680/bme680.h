#ifndef _BME680_H_
#define _BME680_H_

#include <stdbool.h>
#include <stdint.h>

#define BME680_STATUS          0x73
#define BME680_RESET           0xE0
#define BME680_ID              0xD0
#define BME680_CONFIG          0x75
#define BME680_CTRL_MEAS       0x74
#define BME680_CTRL_HUM        0x72
#define BME680_CTRL_GAS_1      0x71
#define BME680_CTRL_GAS_0      0x70

#define BME680_GAS_WAIT_0      0x64
#define BME680_RES_HEAT_0      0x5A
#define BME680_IDAC_HEAT_0     0x50

#define BME680_GAS_R_LSB       0x2B
#define BME680_GAS_R_MSB       0x2A
#define BME680_HUM_LSB         0x26
#define BME680_HUM_MSB         0x25
#define BME680_TEMP_XLSB       0x24
#define BME680_TEMP_LSB        0x23
#define BME680_TEMP_MSB        0x22
#define BME680_PRESS_XLSB      0x21
#define BME680_PRESS_LSB       0x20
#define BME680_PRESS_MSB       0x1F
#define BME680_MEAS_STATUS_0   0x1D

#define BME680_I2C_ADDR        0x77
#define BME680_CHIP_ID         0x61


struct bme680_sample {
    int32_t temperature_centi_c;     // 2345 = 23.45 C
    uint32_t humidity_milli_pct;     // 45123 = 45.123 %RH
    uint32_t gas_resistance_ohm;     // gas resistance in ohms
    bool gas_valid;                  // true when gas value can be used
};


int bme680_init(int16_t heater_temp_c, uint16_t heater_duration_ms);
int bme680_read_all(struct bme680_sample *sample);

#endif