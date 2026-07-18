#include "vfr_single_point.h"

#include "voloop_def.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#define VFR_SINGLE_POINT_PHASE_SCALE 4294967296.0
#define VFR_SINGLE_POINT_UINT32_MAX_AS_DOUBLE 4294967295.0
#define VFR_SINGLE_POINT_RAD_TO_DEG (180.0f / VOLOOP_Pi)

static int VFR_SinglePointIsFinitePositive(float value) {
    return isfinite(value) && value > 0.0f;
}

static VFR_SinglePointStatus VFR_SinglePointSetInitError(VFR_SinglePointHandle* handle,
                                                         VFR_SinglePointStatus status) {
    handle->state = VFR_SINGLE_POINT_ERROR;
    handle->status = status;
    return status;
}

static VFR_SinglePointState VFR_SinglePointSetRuntimeError(VFR_SinglePointHandle* handle,
                                                           VFR_SinglePointStatus status,
                                                           float* suggested_modulation) {
    handle->state = VFR_SINGLE_POINT_ERROR;
    handle->status = status;

    if (suggested_modulation != NULL) {
        if (isfinite(handle->config.modulation_bias)) {
            *suggested_modulation = handle->config.modulation_bias;
        } else {
            *suggested_modulation = 0.0f;
        }
    }

    return VFR_SINGLE_POINT_ERROR;
}

static VFR_SinglePointState VFR_SinglePointEmitModulation(VFR_SinglePointHandle* handle,
                                                          float* suggested_modulation) {
    int32_t phase_q31 = (int32_t)handle->phase_accumulator;
    float reference_sine = VOLOOP_DEF_SINQ31(phase_q31);
    float reference_cosine = VOLOOP_DEF_COSQ31(phase_q31);
    float modulation =
        handle->config.modulation_bias + (handle->config.modulation_amplitude * reference_sine);

    if (!isfinite(reference_sine) || !isfinite(reference_cosine) || !isfinite(modulation)) {
        return VFR_SinglePointSetRuntimeError(handle, VFR_SINGLE_POINT_NUMERIC_ERROR,
                                              suggested_modulation);
    }

    handle->reference_sine = reference_sine;
    handle->reference_cosine = reference_cosine;
    handle->last_suggested_modulation = modulation;
    handle->phase_accumulator += handle->phase_step;
    *suggested_modulation = modulation;

    return handle->state;
}

static float VFR_SinglePointWrapPhaseDeg(float phase_deg) {
    if (phase_deg < -180.0f) {
        phase_deg += 360.0f;
    } else if (phase_deg >= 180.0f) {
        phase_deg -= 360.0f;
    }

    return phase_deg;
}

VFR_SinglePointStatus VFR_SinglePointInit(VFR_SinglePointHandle* handle,
                                          const VFR_SinglePointConfig* config) {
    double samples_per_cycle_exact;
    double samples_per_cycle_rounded;
    double warmup_samples_rounded;
    double measure_samples_rounded;
    double total_samples;
    double phase_step;

    if (handle == NULL) {
        return VFR_SINGLE_POINT_INVALID_ARGUMENT;
    }

    *handle = (VFR_SinglePointHandle){ 0 };
    if (config == NULL) {
        return VFR_SinglePointSetInitError(handle, VFR_SINGLE_POINT_INVALID_ARGUMENT);
    }

    handle->config = *config;

    if (!VFR_SinglePointIsFinitePositive(config->sample_rate_hz) ||
        !VFR_SinglePointIsFinitePositive(config->modulation_amplitude) ||
        !VFR_SinglePointIsFinitePositive(config->voltage_abs_limit) ||
        !VFR_SinglePointIsFinitePositive(config->gain_floor) ||
        !isfinite(config->modulation_bias) || config->measure_cycles == 0U ||
        config->min_samples_per_cycle == 0U || config->max_samples_per_point == 0U) {
        return VFR_SinglePointSetInitError(handle, VFR_SINGLE_POINT_INVALID_ARGUMENT);
    }

    if (!isfinite(config->frequency_hz) || config->frequency_hz <= 0.0f ||
        (double)config->frequency_hz >= ((double)config->sample_rate_hz * 0.5)) {
        return VFR_SinglePointSetInitError(handle, VFR_SINGLE_POINT_INVALID_FREQUENCY);
    }

    samples_per_cycle_exact = (double)config->sample_rate_hz / (double)config->frequency_hz;
    samples_per_cycle_rounded = round(samples_per_cycle_exact);
    if (!isfinite(samples_per_cycle_rounded) || samples_per_cycle_rounded < 1.0 ||
        samples_per_cycle_rounded > VFR_SINGLE_POINT_UINT32_MAX_AS_DOUBLE) {
        return VFR_SinglePointSetInitError(handle, VFR_SINGLE_POINT_NUMERIC_ERROR);
    }

    handle->samples_per_cycle = (uint32_t)samples_per_cycle_rounded;
    if (handle->samples_per_cycle < config->min_samples_per_cycle) {
        return VFR_SinglePointSetInitError(handle, VFR_SINGLE_POINT_INSUFFICIENT_SAMPLES);
    }

    warmup_samples_rounded = round(samples_per_cycle_exact * (double)config->warmup_cycles);
    measure_samples_rounded = round(samples_per_cycle_exact * (double)config->measure_cycles);
    total_samples = warmup_samples_rounded + measure_samples_rounded;
    if (!isfinite(warmup_samples_rounded) || !isfinite(measure_samples_rounded) ||
        !isfinite(total_samples) ||
        warmup_samples_rounded > VFR_SINGLE_POINT_UINT32_MAX_AS_DOUBLE ||
        measure_samples_rounded > VFR_SINGLE_POINT_UINT32_MAX_AS_DOUBLE ||
        total_samples > VFR_SINGLE_POINT_UINT32_MAX_AS_DOUBLE ||
        measure_samples_rounded > (double)config->max_samples_per_point) {
        return VFR_SinglePointSetInitError(handle, VFR_SINGLE_POINT_SAMPLE_LIMIT_EXCEEDED);
    }

    phase_step = VFR_SINGLE_POINT_PHASE_SCALE * (double)config->frequency_hz /
                 (double)config->sample_rate_hz;
    if (!isfinite(phase_step) || phase_step < 1.0 || phase_step >= 2147483648.0) {
        return VFR_SinglePointSetInitError(handle, VFR_SINGLE_POINT_NUMERIC_ERROR);
    }

    handle->warmup_samples = (uint32_t)warmup_samples_rounded;
    handle->measure_samples = (uint32_t)measure_samples_rounded;
    handle->phase_step = (uint32_t)phase_step;
    handle->state = VFR_SINGLE_POINT_READY;
    handle->status = VFR_SINGLE_POINT_OK;

    return VFR_SINGLE_POINT_OK;
}

VFR_SinglePointState VFR_SinglePointTick(VFR_SinglePointHandle* handle, float actual_voltage,
                                         float* suggested_modulation) {
    if (handle == NULL) {
        if (suggested_modulation != NULL) {
            *suggested_modulation = 0.0f;
        }
        return VFR_SINGLE_POINT_ERROR;
    }

    if (suggested_modulation == NULL) {
        return VFR_SinglePointSetRuntimeError(handle, VFR_SINGLE_POINT_INVALID_ARGUMENT, NULL);
    }

    if (handle->state == VFR_SINGLE_POINT_COMPLETE) {
        *suggested_modulation = handle->config.modulation_bias;
        return handle->state;
    }

    if (handle->state == VFR_SINGLE_POINT_ERROR) {
        if (isfinite(handle->config.modulation_bias)) {
            *suggested_modulation = handle->config.modulation_bias;
        } else {
            *suggested_modulation = 0.0f;
        }
        return handle->state;
    }

    if (handle->state == VFR_SINGLE_POINT_READY) {
        if (handle->warmup_samples > 0U) {
            handle->state = VFR_SINGLE_POINT_WARMUP;
        } else {
            handle->state = VFR_SINGLE_POINT_MEASURING;
        }
        return VFR_SinglePointEmitModulation(handle, suggested_modulation);
    }

    if (handle->state != VFR_SINGLE_POINT_WARMUP && handle->state != VFR_SINGLE_POINT_MEASURING) {
        return VFR_SinglePointSetRuntimeError(handle, VFR_SINGLE_POINT_INVALID_STATE,
                                              suggested_modulation);
    }

    if (!isfinite(actual_voltage)) {
        return VFR_SinglePointSetRuntimeError(handle, VFR_SINGLE_POINT_NUMERIC_ERROR,
                                              suggested_modulation);
    }
    if (fabsf(actual_voltage) > handle->config.voltage_abs_limit) {
        return VFR_SinglePointSetRuntimeError(handle, VFR_SINGLE_POINT_VOLTAGE_LIMIT_EXCEEDED,
                                              suggested_modulation);
    }

    if (handle->state == VFR_SINGLE_POINT_WARMUP) {
        ++handle->warmup_index;
        if (handle->warmup_index >= handle->warmup_samples) {
            handle->state = VFR_SINGLE_POINT_MEASURING;
        }
    } else {
        handle->sine_accumulator += actual_voltage * handle->reference_sine;
        handle->cosine_accumulator += actual_voltage * handle->reference_cosine;
        ++handle->measure_index;

        if (!isfinite(handle->sine_accumulator) || !isfinite(handle->cosine_accumulator)) {
            return VFR_SinglePointSetRuntimeError(handle, VFR_SINGLE_POINT_NUMERIC_ERROR,
                                                  suggested_modulation);
        }

        if (handle->measure_index >= handle->measure_samples) {
            handle->state = VFR_SINGLE_POINT_COMPLETE;
            handle->status = VFR_SINGLE_POINT_OK;
            handle->last_suggested_modulation = handle->config.modulation_bias;
            *suggested_modulation = handle->config.modulation_bias;
            return handle->state;
        }
    }

    return VFR_SinglePointEmitModulation(handle, suggested_modulation);
}

VFR_SinglePointState VFR_SinglePointGetState(const VFR_SinglePointHandle* handle) {
    if (handle == NULL) {
        return VFR_SINGLE_POINT_ERROR;
    }

    return handle->state;
}

VFR_SinglePointStatus VFR_SinglePointGetStatus(const VFR_SinglePointHandle* handle) {
    if (handle == NULL) {
        return VFR_SINGLE_POINT_INVALID_ARGUMENT;
    }

    return handle->status;
}

VFR_SinglePointStatus VFR_SinglePointGetResult(const VFR_SinglePointHandle* handle,
                                               VFR_SinglePointResult* result) {
    float normalization;

    if (result != NULL) {
        *result = (VFR_SinglePointResult){ 0 };
    }

    if (handle == NULL || result == NULL) {
        return VFR_SINGLE_POINT_INVALID_ARGUMENT;
    }

    result->frequency_hz = handle->config.frequency_hz;
    result->modulation_amplitude = handle->config.modulation_amplitude;
    result->warmup_samples = handle->warmup_samples;
    result->measure_samples = handle->measure_samples;
    result->total_samples = handle->warmup_samples + handle->measure_samples;

    if (handle->state != VFR_SINGLE_POINT_COMPLETE) {
        result->status = VFR_SINGLE_POINT_INVALID_STATE;
        return result->status;
    }

    normalization = 2.0f / (float)handle->measure_samples;
    result->synchronous_accumulator = handle->sine_accumulator;
    result->quadrature_accumulator = handle->cosine_accumulator;
    result->synchronous_component = normalization * result->synchronous_accumulator;
    result->quadrature_component = normalization * result->quadrature_accumulator;
    result->voltage_amplitude =
        sqrtf((result->synchronous_component * result->synchronous_component) +
              (result->quadrature_component * result->quadrature_component));
    result->gain_linear = result->voltage_amplitude / handle->config.modulation_amplitude;
    result->phase_deg = VFR_SinglePointWrapPhaseDeg(
        atan2f(result->quadrature_component, result->synchronous_component) *
        VFR_SINGLE_POINT_RAD_TO_DEG);

    if (!isfinite(result->synchronous_accumulator) || !isfinite(result->quadrature_accumulator) ||
        !isfinite(result->synchronous_component) || !isfinite(result->quadrature_component) ||
        !isfinite(result->voltage_amplitude) || !isfinite(result->gain_linear) ||
        !isfinite(result->phase_deg)) {
        result->status = VFR_SINGLE_POINT_NUMERIC_ERROR;
        return result->status;
    }

    if (result->gain_linear < handle->config.gain_floor) {
        result->gain_linear = handle->config.gain_floor;
        result->gain_db = 20.0f * log10f(result->gain_linear);
        if (!isfinite(result->gain_db)) {
            result->status = VFR_SINGLE_POINT_NUMERIC_ERROR;
            return result->status;
        }
        result->status = VFR_SINGLE_POINT_OK_WITH_GAIN_FLOOR;
        return result->status;
    }

    result->gain_db = 20.0f * log10f(result->gain_linear);
    if (!isfinite(result->gain_db)) {
        result->status = VFR_SINGLE_POINT_NUMERIC_ERROR;
        return result->status;
    }

    result->status = VFR_SINGLE_POINT_OK;
    return result->status;
}

const char* VFR_SinglePointStatusToString(VFR_SinglePointStatus status) {
    switch (status) {
        case VFR_SINGLE_POINT_OK:
            return "ok";
        case VFR_SINGLE_POINT_OK_WITH_GAIN_FLOOR:
            return "ok_with_gain_floor";
        case VFR_SINGLE_POINT_INVALID_ARGUMENT:
            return "invalid_argument";
        case VFR_SINGLE_POINT_INVALID_FREQUENCY:
            return "invalid_frequency";
        case VFR_SINGLE_POINT_INSUFFICIENT_SAMPLES:
            return "insufficient_samples";
        case VFR_SINGLE_POINT_SAMPLE_LIMIT_EXCEEDED:
            return "sample_limit_exceeded";
        case VFR_SINGLE_POINT_VOLTAGE_LIMIT_EXCEEDED:
            return "voltage_limit_exceeded";
        case VFR_SINGLE_POINT_NUMERIC_ERROR:
            return "numeric_error";
        case VFR_SINGLE_POINT_INVALID_STATE:
            return "invalid_state";
        default:
            return "unknown";
    }
}
