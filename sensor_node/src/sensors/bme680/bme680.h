#ifndef _BME680_H_
#define _BME680_H_

#include <stdbool.h>
#include <stdint.h>

struct bme680_sample {
    int32_t temperature_centi_c;     // 2345 = 23.45 C
    uint32_t humidity_milli_pct;     // 45123 = 45.123 %RH
    uint32_t gas_resistance_ohm;     // gas resistance in ohms
    bool gas_valid;                  // true when gas value can be used
};

int bme680_init(int16_t heater_temp_c, uint16_t heater_duration_ms);
int bme680_read_all(struct bme680_sample *sample);

#endif
