#ifndef BASE_STATION_REMOTE_PICO_INTERRUPTS_H_
#define BASE_STATION_REMOTE_PICO_INTERRUPTS_H_

#include <stdbool.h>
#include <zephyr/device.h>

#define REMOTE_PICO_FIRE_THRESHOLD_CENTI_C 3000

int remote_pico_interrupts_init(const struct device *sensor);
void remote_pico_interrupts_try_configure(bool force);
void remote_pico_interrupts_lock_sensor(void);
void remote_pico_interrupts_unlock_sensor(void);

#endif /* BASE_STATION_REMOTE_PICO_INTERRUPTS_H_ */
