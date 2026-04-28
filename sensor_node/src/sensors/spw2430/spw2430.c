#include "spw2430.h"

#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <errno.h>
#include <math.h>

#define ADC_NODE DT_PATH(zephyr_user)
static const struct adc_dt_spec mic_adc = ADC_DT_SPEC_GET_BY_IDX(ADC_NODE, 0);

static int16_t sample_buf;
static struct adc_sequence seq;

static int32_t baseline = 0;
static bool initialized = false;

int spw2430_init(void)
{
    int ret;

    if (!adc_is_ready_dt(&mic_adc)) {
        return -ENODEV;
    }

    ret = adc_channel_setup_dt(&mic_adc);
    if (ret < 0) {
        return ret;
    }

    seq.buffer = &sample_buf;
    seq.buffer_size = sizeof(sample_buf);

    ret = adc_sequence_init_dt(&mic_adc, &seq);
    if (ret < 0) {
        return ret;
    }

    initialized = true;
    return 0;
}

int spw2430_read_raw(int16_t *raw)
{
    int ret;

    if (!initialized || raw == NULL) {
        return -EINVAL;
    }

    ret = adc_read_dt(&mic_adc, &seq);
    if (ret < 0) {
        return ret;
    }

    *raw = sample_buf;
    return 0;
}

int spw2430_calibrate(int samples)
{
    int ret;
    int16_t raw;
    int64_t sum = 0;

    if (!initialized || samples <= 0) {
        return -EINVAL;
    }

    for (int i = 0; i < samples; i++) {
        ret = spw2430_read_raw(&raw);
        if (ret < 0) {
            return ret;
        }

        sum += raw;
    }

    baseline = sum / samples;
    return 0;
}

int spw2430_read_level(int samples, int32_t *peak, int32_t *rms)
{
    int ret;
    int16_t raw;
    int32_t max = 0;
    int64_t sum_sq = 0;

    if (!initialized || samples <= 0) {
        return -EINVAL;
    }

    if (peak == NULL && rms == NULL) {
        return -EINVAL;
    }

    for (int i = 0; i < samples; i++) {
        ret = spw2430_read_raw(&raw);
        if (ret < 0) {
            return ret;
        }

        int32_t diff = raw - baseline;
        int32_t abs_diff = diff < 0 ? -diff : diff;

        if (abs_diff > max) {
            max = abs_diff;
        }

        sum_sq += diff * diff;
    }

    if (peak != NULL) {
        *peak = max;
    }

    if (rms != NULL) {
        *rms = sqrtf((float)(sum_sq / samples));
    }

    return 0;
}

int32_t spw2430_get_baseline(void)
{
    return baseline;
}