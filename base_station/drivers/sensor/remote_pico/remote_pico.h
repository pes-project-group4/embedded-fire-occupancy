#ifndef REMOTE_PICO_CHANNELS_H_
#define REMOTE_PICO_CHANNELS_H_

#include <zephyr/drivers/sensor.h>

/*
 * Custom sensor channels exposed by the remote_pico driver.
 *
 * Standard Zephyr channels we also support (use directly):
 *   SENSOR_CHAN_AMBIENT_TEMP  — air temperature (BME680)
 *   SENSOR_CHAN_HUMIDITY      — relative humidity (BME680)
 *   SENSOR_CHAN_GAS_RES       — gas resistance in ohm (BME680)
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

#endif /* REMOTE_PICO_CHANNELS_H_ */
