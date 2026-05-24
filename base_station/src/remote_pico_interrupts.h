#ifndef BASE_STATION_REMOTE_PICO_INTERRUPTS_H_
#define BASE_STATION_REMOTE_PICO_INTERRUPTS_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>

#define REMOTE_PICO_FIRE_THRESHOLD_DEFAULT_CENTI_C 3000

int remote_pico_interrupts_init(const struct device *sensor);
int remote_pico_interrupts_set_fire_threshold(int32_t centi_c);
int32_t remote_pico_interrupts_get_fire_threshold(void);
void remote_pico_interrupts_try_configure(bool force);
void remote_pico_interrupts_lock_sensor(void);
void remote_pico_interrupts_unlock_sensor(void);

#endif /* BASE_STATION_REMOTE_PICO_INTERRUPTS_H_ */
