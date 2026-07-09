#include "vfr_point_measure.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>

#define VFR_PI 3.141592653589793238462643383279502884
#define VFR_TWO_PI (2.0 * VFR_PI)
#define VFR_RAD_TO_DEG (180.0 / VFR_PI)
#define VFR_UINT32_MAX_AS_DOUBLE 4294967295.0

static int vfr_is_finite_positive(double value) {
    return isfinite(value) && value > 0.0;
}

static double vfr_wrap_phase_deg(double phase_deg) {
    while (phase_deg < -180.0) {
        phase_deg += 360.0;
    }
    while (phase_deg >= 180.0) {
        phase_deg -= 360.0;
    }
    return phase_deg;
}

static VFR_PointMeasureStatus vfr_finish(VFR_PointMeasureResult* result,
                                         VFR_PointMeasureStatus status) {
    if (result != NULL) {
        result->status = status;
    }
    return status;
}

const char* VFR_PointMeasureStatusToString(VFR_PointMeasureStatus status) {
    switch (status) {
        case VFR_POINT_OK:
            return "ok";
        case VFR_POINT_OK_WITH_GAIN_FLOOR:
            return "ok_with_gain_floor";
        case VFR_POINT_INVALID_ARGUMENT:
            return "invalid_argument";
        case VFR_POINT_INVALID_FREQUENCY:
            return "invalid_frequency";
        case VFR_POINT_INSUFFICIENT_SAMPLES:
            return "insufficient_samples";
        case VFR_POINT_SAMPLE_LIMIT_EXCEEDED:
            return "sample_limit_exceeded";
        case VFR_POINT_OUTPUT_LIMIT_EXCEEDED:
            return "output_limit_exceeded";
        case VFR_POINT_NUMERIC_ERROR:
            return "numeric_error";
        case VFR_POINT_SUBJECT_ERROR:
            return "subject_error";
        default:
            return "unknown";
    }
}

VFR_PointMeasureStatus VFR_MeasurePoint(VFR_TestSubject* subject,
                                        const VFR_PointMeasureConfig* config,
                                        VFR_PointMeasureResult* result) {
    double samples_per_cycle_exact;
    double samples_per_cycle_rounded;
    uint32_t samples_per_cycle;
    double warmup_samples_d;
    double measure_samples_d;
    double total_samples_d;
    uint32_t warmup_samples;
    uint32_t measure_samples;
    uint32_t total_samples;
    double theta_step;
    double theta;
    double sin_accum;
    double cos_accum;
    uint32_t i;

    if (result != NULL) {
        *result = (VFR_PointMeasureResult){0};
    }

    if (subject == NULL || config == NULL || result == NULL || subject->compute == NULL) {
        return vfr_finish(result, VFR_POINT_INVALID_ARGUMENT);
    }

    result->frequency_hz = config->frequency_hz;
    result->input_amplitude = config->input_amplitude;

    if (!vfr_is_finite_positive(config->sample_rate_hz) ||
        !vfr_is_finite_positive(config->input_amplitude) ||
        !vfr_is_finite_positive(config->output_abs_limit) ||
        !vfr_is_finite_positive(config->gain_floor) ||
        config->measure_cycles == 0U ||
        config->min_samples_per_cycle == 0U ||
        config->max_samples_per_point == 0U) {
        return vfr_finish(result, VFR_POINT_INVALID_ARGUMENT);
    }

    if (!isfinite(config->frequency_hz) || config->frequency_hz <= 0.0 ||
        config->frequency_hz >= (config->sample_rate_hz * 0.5)) {
        return vfr_finish(result, VFR_POINT_INVALID_FREQUENCY);
    }

    samples_per_cycle_exact = config->sample_rate_hz / config->frequency_hz;
    samples_per_cycle_rounded = round(samples_per_cycle_exact);
    if (!isfinite(samples_per_cycle_rounded) || samples_per_cycle_rounded < 1.0 ||
        samples_per_cycle_rounded > VFR_UINT32_MAX_AS_DOUBLE) {
        return vfr_finish(result, VFR_POINT_NUMERIC_ERROR);
    }

    /*
     * Test-point sampling-period approximation:
     * use round(fs / f) so warmup and measurement cover approximately integer
     * input cycles for the requested frequency point. Stage-one/two tests
     * should prefer frequencies that divide the sample rate closely. Future
     * logarithmic sweeps should evaluate leakage using:
     * actual_cycles_measured = measure_samples * frequency_hz / sample_rate_hz.
     */
    samples_per_cycle = (uint32_t)samples_per_cycle_rounded;
    if (samples_per_cycle < config->min_samples_per_cycle) {
        return vfr_finish(result, VFR_POINT_INSUFFICIENT_SAMPLES);
    }

    warmup_samples_d = (double)samples_per_cycle * (double)config->warmup_cycles;
    measure_samples_d = (double)samples_per_cycle * (double)config->measure_cycles;
    total_samples_d = warmup_samples_d + measure_samples_d;
    if (warmup_samples_d > VFR_UINT32_MAX_AS_DOUBLE ||
        measure_samples_d > VFR_UINT32_MAX_AS_DOUBLE ||
        total_samples_d > VFR_UINT32_MAX_AS_DOUBLE) {
        return vfr_finish(result, VFR_POINT_SAMPLE_LIMIT_EXCEEDED);
    }

    warmup_samples = (uint32_t)warmup_samples_d;
    measure_samples = (uint32_t)measure_samples_d;
    total_samples = (uint32_t)total_samples_d;
    result->warmup_samples = warmup_samples;
    result->measure_samples = measure_samples;
    result->total_samples = total_samples;

    if (total_samples > config->max_samples_per_point) {
        return vfr_finish(result, VFR_POINT_SAMPLE_LIMIT_EXCEEDED);
    }

    if (subject->reset != NULL) {
        subject->reset(subject->context);
    }

    theta_step = VFR_TWO_PI * config->frequency_hz / config->sample_rate_hz;
    theta = 0.0;
    for (i = 0U; i < warmup_samples; ++i) {
        double input = config->input_amplitude * sin(theta);
        double output = (double)subject->compute(subject->context, (float)input);

        if (!isfinite(output)) {
            return vfr_finish(result, VFR_POINT_NUMERIC_ERROR);
        }
        if (fabs(output) > config->output_abs_limit) {
            return vfr_finish(result, VFR_POINT_OUTPUT_LIMIT_EXCEEDED);
        }

        theta += theta_step;
    }

    sin_accum = 0.0;
    cos_accum = 0.0;
    for (i = 0U; i < measure_samples; ++i) {
        double sin_theta = sin(theta);
        double cos_theta = cos(theta);
        double input = config->input_amplitude * sin_theta;
        double output = (double)subject->compute(subject->context, (float)input);

        if (!isfinite(output)) {
            return vfr_finish(result, VFR_POINT_NUMERIC_ERROR);
        }
        if (fabs(output) > config->output_abs_limit) {
            return vfr_finish(result, VFR_POINT_OUTPUT_LIMIT_EXCEEDED);
        }

        sin_accum += output * sin_theta;
        cos_accum += output * cos_theta;
        theta += theta_step;
    }

    result->output_amplitude =
        (2.0 / (double)measure_samples) * sqrt((sin_accum * sin_accum) + (cos_accum * cos_accum));
    result->gain_linear = result->output_amplitude / config->input_amplitude;
    result->phase_deg = vfr_wrap_phase_deg(atan2(cos_accum, sin_accum) * VFR_RAD_TO_DEG);

    if (!isfinite(result->output_amplitude) || !isfinite(result->gain_linear) ||
        !isfinite(result->phase_deg)) {
        return vfr_finish(result, VFR_POINT_NUMERIC_ERROR);
    }

    if (result->gain_linear < config->gain_floor) {
        result->gain_linear = config->gain_floor;
        result->gain_db = 20.0 * log10(result->gain_linear);
        if (!isfinite(result->gain_db)) {
            return vfr_finish(result, VFR_POINT_NUMERIC_ERROR);
        }
        return vfr_finish(result, VFR_POINT_OK_WITH_GAIN_FLOOR);
    }

    result->gain_db = 20.0 * log10(result->gain_linear);
    if (!isfinite(result->gain_db)) {
        return vfr_finish(result, VFR_POINT_NUMERIC_ERROR);
    }

    return vfr_finish(result, VFR_POINT_OK);
}
