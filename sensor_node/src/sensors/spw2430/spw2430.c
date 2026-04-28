#include "spw2430.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>
#include <errno.h>
#include <math.h>

LOG_MODULE_REGISTER(spw2430, CONFIG_LOG_DEFAULT_LEVEL);

#define ADC_NODE DT_PATH(zephyr_user)

BUILD_ASSERT(DT_NODE_HAS_PROP(ADC_NODE, io_channels), "zephyr,user node must declare io-channels for SPW2430");

static const struct adc_dt_spec mic_adc = ADC_DT_SPEC_GET_BY_IDX(ADC_NODE, 0);

static int16_t sample_buf;
static struct adc_sequence seq;

static int32_t baseline;
static bool calibrated;
static bool initialized;

int spw2430_init(void) {
    int ret;

    if (initialized) {
        return 0;
    }

    if (!adc_is_ready_dt(&mic_adc)) {
        LOG_ERR("ADC device not ready");
        return -ENODEV;
    }

    ret = adc_channel_setup_dt(&mic_adc);
    if (ret < 0) {
        LOG_ERR("ADC channel setup failed: %d", ret);
        return ret;
    }

    seq.buffer = &sample_buf;
    seq.buffer_size = sizeof(sample_buf);

    ret = adc_sequence_init_dt(&mic_adc, &seq);
    if (ret < 0) {
        LOG_ERR("ADC sequence init failed: %d", ret);
        return ret;
    }

    baseline = 0;
    calibrated = false;
    initialized = true;

    LOG_INF("SPW2430 initialized (resolution=%u)", mic_adc.resolution);
    return 0;
}

int spw2430_read_raw(int16_t *raw) {
    int ret;

    if (!initialized) {
        return -EPERM;
    }
    if (raw == NULL) {
        return -EINVAL;
    }

    ret = adc_read_dt(&mic_adc, &seq);
    if (ret < 0) {
        return ret;
    }

    *raw = sample_buf;
    return 0;
}

int spw2430_calibrate(int samples) {
    int ret;
    int16_t raw;
    int64_t sum = 0;

    if (!initialized) {
        return -EPERM;
    }
    if (samples <= 0) {
        return -EINVAL;
    }

    //discard initial samples to let the ADC settle
    for (int i = 0; i < SPW2430_CALIB_DISCARD_SAMPLES; i++) {
        ret = spw2430_read_raw(&raw);
        if (ret < 0) {
            LOG_ERR("Calibration discard read failed: %d", ret);
            return ret;
        }
    }

    for (int i = 0; i < samples; i++) {
        ret = spw2430_read_raw(&raw);
        if (ret < 0) {
            LOG_ERR("Calibration read failed: %d", ret);
            return ret;
        }
        sum += raw;
    }

    baseline = (int32_t)(sum / samples);
    calibrated = true;

    LOG_INF("Calibrated: baseline=%d (over %d samples)",
            (int)baseline, samples);
    return 0;
}

int spw2430_read_level(int samples, int32_t *peak_out, int32_t *rms_out) {
    int ret;
    int16_t raw;
    int32_t peak = 0;
    int64_t sum_sq = 0;

    if (!initialized) {
        return -EPERM;
    }
    if (samples <= 0) {
        return -EINVAL;
    }
    if (peak_out == NULL && rms_out == NULL) {
        return -EINVAL;
    }

    for (int i = 0; i < samples; i++) {
        ret = spw2430_read_raw(&raw);
        if (ret < 0) {
            return ret;
        }

        int32_t diff = (int32_t)raw - baseline;
        int32_t adiff = (diff < 0) ? -diff : diff;

        if (adiff > peak) {
            peak = adiff;
        }
        sum_sq += (int64_t)diff * diff;
    }

    if (peak_out != NULL) {
        *peak_out = peak;
    }
    if (rms_out != NULL) {
        *rms_out = (int32_t)sqrtf((float)(sum_sq / samples));
    }

    return 0;
}

int spw2430_read_rms(int samples, int32_t *rms_out) {
    if (rms_out == NULL) {
        return -EINVAL;
    }
    return spw2430_read_level(samples, NULL, rms_out);
}

int spw2430_read_peak(int samples, int32_t *peak_out) {
    if (peak_out == NULL) {
        return -EINVAL;
    }
    return spw2430_read_level(samples, peak_out, NULL);
}

int32_t spw2430_get_baseline(void) {
    return baseline;
}

bool spw2430_is_calibrated(void) {
    return calibrated;
}