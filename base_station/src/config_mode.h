#ifndef BASE_STATION_CONFIG_MODE_H_
#define BASE_STATION_CONFIG_MODE_H_

#include <stdbool.h>
#include <zephyr/device.h>

void config_mode_run(const struct device *sensor,
                     bool (*exit_requested)(void));

#endif /* BASE_STATION_CONFIG_MODE_H_ */
