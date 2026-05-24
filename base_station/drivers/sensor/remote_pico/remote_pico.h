#ifndef REMOTE_PICO_CHANNELS_H_
#define REMOTE_PICO_CHANNELS_H_

#include <zephyr/drivers/sensor.h>
#include <stdint.h>

/*
 * Custom sensor channels exposed by the remote_pico driver.
 *
 * Standard Zephyr channels we also support (use directly):
 *   SENSOR_CHAN_AMBIENT_TEMP  - air temperature (BME680)
 *   SENSOR_CHAN_HUMIDITY      - relative humidity (BME680)
 *   SENSOR_CHAN_GAS_RES       - gas resistance in ohm (BME680)
 *
 * Custom channels for everything else live here. Numbering starts at
 * SENSOR_CHAN_PRIV_START as required by Zephyr.
 */
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
    REMOTE_PICO_INT_MIC_PEAK   = (1u << 0),
    REMOTE_PICO_INT_MIC_RMS    = (1u << 1),
    REMOTE_PICO_INT_T_OBJ_HIGH = (1u << 2),
    REMOTE_PICO_INT_T_AIR_HIGH = (1u << 3),
    REMOTE_PICO_INT_MMWAVE     = (1u << 4),
};

#define REMOTE_PICO_THRESHOLD_DISABLED INT32_MAX

int remote_pico_set_interrupts(const struct device *dev, uint8_t mask);
int remote_pico_clear_interrupts(const struct device *dev);
int remote_pico_set_object_temp_high(const struct device *dev, int32_t centi_c);
int remote_pico_set_air_temp_high(const struct device *dev, int32_t centi_c);
int remote_pico_set_mic_peak_threshold(const struct device *dev, int32_t adc_counts);
int remote_pico_set_mic_rms_threshold(const struct device *dev, int32_t adc_counts);
int remote_pico_set_mmwave_max_gate(const struct device *dev, uint8_t max_gate);
int remote_pico_set_mmwave_absence_delay(const struct device *dev,
                                         uint16_t seconds);

#endif /* REMOTE_PICO_CHANNELS_H_ */
