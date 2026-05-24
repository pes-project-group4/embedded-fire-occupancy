#ifndef BASE_STATION_FIRE_ALARM_H_
#define BASE_STATION_FIRE_ALARM_H_

#include <stdbool.h>

int fire_alarm_init(void);
void fire_alarm_trigger(bool occupied);
void fire_alarm_stop(void);

#endif /* BASE_STATION_FIRE_ALARM_H_ */
