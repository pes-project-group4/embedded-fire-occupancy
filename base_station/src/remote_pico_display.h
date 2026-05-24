#ifndef BASE_STATION_REMOTE_PICO_DISPLAY_H_
#define BASE_STATION_REMOTE_PICO_DISPLAY_H_

#include <stdint.h>
#include <zephyr/device.h>

void remote_pico_print_irq_sources(uint8_t src);
void remote_pico_print_sensor_snapshot(const struct device *sensor);

#endif
