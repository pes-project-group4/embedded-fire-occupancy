#ifndef _MLX90614_H_
#define _MLX90614_H_

#include <stdbool.h>
#include <stdint.h>

// RAM registers
#define MLX90614_RAM_RAW_IR1 0x04
#define MLX90614_RAM_RAW_IR2 0x05
#define MLX90614_RAM_TA 0x06
#define MLX90614_RAM_TOBJ1 0x07
#define MLX90614_RAM_TOBJ2 0x08

// EEPROM registers
#define MLX90614_EEPROM_EMISSIVITY 0x24
#define MLX90614_EEPROM_CONFIG 0x25
#define MLX90614_EEPROM_ADDR 0x2E
#define MLX90614_EEPROM_ID_1 0x3C

#define MLX90614_I2C_ADDR 0x5A

// bit 15 means sensor error
#define MLX90614_ERROR_FLAG_BIT 0x8000

struct mlx90614_sample {
    int32_t ambient_centi_c;
    int32_t object_centi_c;
};

int mlx90614_init(void);

int mlx90614_read(struct mlx90614_sample *sample);

#endif