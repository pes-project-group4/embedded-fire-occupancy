#ifndef SPW2430_H
#define SPW2430_H

#include <stdint.h>
#include <stdbool.h>

#define SPW2430_DEFAULT_CALIB_SAMPLES 500
#define SPW2430_DEFAULT_WINDOW_SAMPLES 64

int spw2430_init(void);
int spw2430_read_raw(int16_t *raw);
int spw2430_calibrate(int samples);
int spw2430_read_level(int samples, int32_t *peak, int32_t *rms);
int32_t spw2430_get_baseline(void);

#endif