#ifndef _I2C_TARGET_H_
#define _I2C_TARGET_H_

#include <stdint.h>

/*
 * I2C target on i2c1, exposing the register map to the base station.
 *
 * Call i2c_target_start() once after regmap_init().
 * Address is the 7-bit I2C address the base station should use (e.g. 0x42).
 */
int i2c_target_start(uint8_t address);

#endif
