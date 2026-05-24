#ifndef REMOTE_PICO_CHANNELS_H_
#define REMOTE_PICO_CHANNELS_H_

#include <zephyr/drivers/sensor.h>
#include <stdint.h>

enum remote_pico_chan {
    REMOTE_PICO_CHAN_OBJECT_TEMP = SENSOR_CHAN_PRIV_START,
    REMOTE_PICO_CHAN_MMWAVE_RANGE,
    REMOTE_PICO_CHAN_OCCUPANCY,
    REMOTE_PICO_CHAN_MIC_PEAK,
    REMOTE_PICO_CHAN_MIC_RMS,
    REMOTE_PICO_CHAN_GAS_VALID,
    REMOTE_PICO_CHAN_INT_SRC,
};

enum remote_pico_int_source {
    REMOTE_PICO_INT_T_OBJ_HIGH = (1u << 2),
};

int remote_pico_set_interrupts(const struct device *dev, uint8_t mask);
int remote_pico_clear_interrupts(const struct device *dev);
int remote_pico_set_object_temp_high(const struct device *dev, int32_t centi_c);
int remote_pico_set_mmwave_max_gate(const struct device *dev, uint8_t max_gate);
int remote_pico_set_mmwave_absence_delay(const struct device *dev, uint16_t seconds);

#endif
