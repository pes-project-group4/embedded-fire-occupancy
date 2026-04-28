#ifndef SPW2430_H
#define SPW2430_H

#include <stdint.h>
#include <stdbool.h>

/* tuning parameters */
#define SPW2430_DEFAULT_CALIB_SAMPLES   500
#define SPW2430_DEFAULT_WINDOW_SAMPLES  256
#define SPW2430_CALIB_DISCARD_SAMPLES   32

int spw2430_init(void);

int spw2430_calibrate(int samples);

int spw2430_read_raw(int16_t *raw);
int spw2430_read_rms(int samples, int32_t *rms_out);
int spw2430_read_peak(int samples, int32_t *peak_out);
int spw2430_read_level(int samples, int32_t *peak_out, int32_t *rms_out);

int32_t spw2430_get_baseline(void);
bool spw2430_is_calibrated(void);

#endif /* SPW2430_H */