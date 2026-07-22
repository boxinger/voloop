#include "vfr_multi_point.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#define VFR_MULTI_POINT_UINT32_MAX_AS_DOUBLE 4294967295.0

static int VFR_MultiPointIsFinitePositive(float value) {
    return isfinite(value) && value > 0.0f;
}

static void VFR_MultiPointDisableOutput(VFR_MultiPointOutput* output) {
    if (output != NULL) {
        output->pwm_state = VOLOOP_PWM_DISABLED;
        output->modulation = 0.0f;
    }
}

static void VFR_MultiPointCopyOutput(VFR_MultiPointOutput* output,
                                     const VFR_SinglePointOutput* point_output) {
    output->pwm_state = point_output->pwm_state;
    output->modulation = point_output->modulation;
}

static VFR_MultiPointStatus VFR_MultiPointSetInitError(VFR_MultiPointHandle* handle,
                                                       VFR_MultiPointStatus status) {
    handle->state = VFR_MULTI_POINT_ERROR;
    handle->status = status;
    return status;
}

static VFR_MultiPointStatus VFR_MultiPointMapFatalStatus(VFR_SinglePointStatus point_status) {
    switch (point_status) {
        case VFR_SINGLE_POINT_INVALID_ARGUMENT:
            return VFR_MULTI_POINT_INVALID_ARGUMENT;
        case VFR_SINGLE_POINT_VOLTAGE_LIMIT_EXCEEDED:
            return VFR_MULTI_POINT_VOLTAGE_LIMIT_EXCEEDED;
        case VFR_SINGLE_POINT_NUMERIC_ERROR:
            return VFR_MULTI_POINT_NUMERIC_ERROR;
        case VFR_SINGLE_POINT_INVALID_STATE:
            return VFR_MULTI_POINT_INVALID_STATE;
        default:
            return VFR_MULTI_POINT_INVALID_STATE;
    }
}

static int VFR_MultiPointIsSkippableInitStatus(VFR_SinglePointStatus point_status) {
    return point_status == VFR_SINGLE_POINT_INVALID_FREQUENCY ||
           point_status == VFR_SINGLE_POINT_INSUFFICIENT_SAMPLES ||
           point_status == VFR_SINGLE_POINT_SAMPLE_LIMIT_EXCEEDED ||
           point_status == VFR_SINGLE_POINT_NUMERIC_ERROR;
}

static VFR_MultiPointState VFR_MultiPointSetFatalError(VFR_MultiPointHandle* handle,
                                                       VFR_SinglePointStatus point_status,
                                                       int record_current_point,
                                                       VFR_MultiPointOutput* output) {
    if (record_current_point && handle->current_index < handle->config.frequency_count) {
        VFR_MultiPointResult* result = &handle->config.results[handle->current_index];

        result->synchronous_component = NAN;
        result->quadrature_component = NAN;
        result->status = point_status;
        ++handle->processed_count;
    }

    handle->state = VFR_MULTI_POINT_ERROR;
    handle->status = VFR_MultiPointMapFatalStatus(point_status);
    VFR_MultiPointDisableOutput(output);
    return handle->state;
}

static VFR_SinglePointConfig VFR_MultiPointMakePointConfig(const VFR_MultiPointHandle* handle,
                                                           uint32_t measure_cycles) {
    VFR_SinglePointConfig point_config = { 0 };

    point_config.sample_rate_hz = handle->config.sample_rate_hz;
    point_config.frequency_hz = handle->config.frequencies_hz[handle->current_index];
    point_config.modulation_bias = handle->config.modulation_bias;
    point_config.modulation_amplitude = handle->config.modulation_amplitude;
    point_config.measure_cycles = measure_cycles;
    point_config.warmup_cycles = (measure_cycles / 2U) + (measure_cycles % 2U);
    point_config.min_samples_per_cycle = handle->config.min_samples_per_cycle;
    point_config.max_samples_per_point = handle->config.max_samples_per_point;
    point_config.voltage_abs_limit = handle->config.voltage_abs_limit;
    point_config.gain_floor = handle->config.gain_floor;

    return point_config;
}

static int VFR_MultiPointCyclesFit(double samples_per_cycle_exact, uint32_t measure_cycles,
                                   uint32_t max_samples_per_point) {
    uint32_t warmup_cycles = (measure_cycles / 2U) + (measure_cycles % 2U);
    double measure_samples = round(samples_per_cycle_exact * (double)measure_cycles);
    double warmup_samples = round(samples_per_cycle_exact * (double)warmup_cycles);
    double total_samples = measure_samples + warmup_samples;

    return isfinite(measure_samples) && isfinite(warmup_samples) && isfinite(total_samples) &&
           measure_samples >= 1.0 && measure_samples <= (double)max_samples_per_point &&
           warmup_samples <= VFR_MULTI_POINT_UINT32_MAX_AS_DOUBLE &&
           total_samples <= VFR_MULTI_POINT_UINT32_MAX_AS_DOUBLE;
}

static uint32_t VFR_MultiPointFindMeasureCycles(const VFR_MultiPointHandle* handle) {
    double samples_per_cycle_exact = (double)handle->config.sample_rate_hz /
                                     (double)handle->config.frequencies_hz[handle->current_index];
    uint32_t low = 1U;
    uint32_t high = handle->config.max_samples_per_point;

    while (low < high) {
        uint32_t middle = low + (uint32_t)(((uint64_t)high - (uint64_t)low + 1ULL) / 2ULL);

        if (VFR_MultiPointCyclesFit(samples_per_cycle_exact, middle,
                                    handle->config.max_samples_per_point)) {
            low = middle;
        } else {
            high = middle - 1U;
        }
    }

    return low;
}

static VFR_MultiPointState VFR_MultiPointFinishSkippedPoint(VFR_MultiPointHandle* handle,
                                                            VFR_SinglePointStatus point_status,
                                                            VFR_MultiPointOutput* output) {
    VFR_MultiPointResult* result = &handle->config.results[handle->current_index];

    result->synchronous_component = NAN;
    result->quadrature_component = NAN;
    result->status = point_status;
    ++handle->processed_count;
    ++handle->skipped_count;
    ++handle->current_index;
    handle->status = VFR_MULTI_POINT_OK_WITH_SKIPPED_POINTS;
    VFR_MultiPointDisableOutput(output);

    if (handle->current_index >= handle->config.frequency_count) {
        handle->state = VFR_MULTI_POINT_COMPLETE;
    } else {
        handle->state = VFR_MULTI_POINT_BETWEEN_POINTS;
    }

    return handle->state;
}

static VFR_MultiPointState VFR_MultiPointMapRunningState(VFR_SinglePointState point_state) {
    if (point_state == VFR_SINGLE_POINT_WARMUP) {
        return VFR_MULTI_POINT_WARMUP;
    }
    if (point_state == VFR_SINGLE_POINT_MEASURING) {
        return VFR_MULTI_POINT_MEASURING;
    }

    return VFR_MULTI_POINT_ERROR;
}

static VFR_MultiPointState VFR_MultiPointStartCurrentPoint(VFR_MultiPointHandle* handle,
                                                           VFR_MultiPointOutput* output) {
    VFR_SinglePointConfig point_config = VFR_MultiPointMakePointConfig(handle, 1U);
    VFR_SinglePointStatus point_status = VFR_SinglePointInit(&handle->point_handle, &point_config);
    VFR_SinglePointOutput point_output = { VOLOOP_PWM_DISABLED, 0.0f };
    VFR_SinglePointState point_state;

    if (point_status != VFR_SINGLE_POINT_OK) {
        if (VFR_MultiPointIsSkippableInitStatus(point_status)) {
            return VFR_MultiPointFinishSkippedPoint(handle, point_status, output);
        }
        return VFR_MultiPointSetFatalError(handle, point_status, 1, output);
    }

    point_config = VFR_MultiPointMakePointConfig(handle, VFR_MultiPointFindMeasureCycles(handle));
    point_status = VFR_SinglePointInit(&handle->point_handle, &point_config);
    if (point_status != VFR_SINGLE_POINT_OK) {
        if (VFR_MultiPointIsSkippableInitStatus(point_status)) {
            return VFR_MultiPointFinishSkippedPoint(handle, point_status, output);
        }
        return VFR_MultiPointSetFatalError(handle, point_status, 1, output);
    }

    point_state = VFR_SinglePointTick(&handle->point_handle, 0.0f, &point_output);
    VFR_MultiPointCopyOutput(output, &point_output);
    if (point_state == VFR_SINGLE_POINT_ERROR) {
        point_status = VFR_SinglePointGetStatus(&handle->point_handle);
        return VFR_MultiPointSetFatalError(handle, point_status, 1, output);
    }

    handle->state = VFR_MultiPointMapRunningState(point_state);
    if (handle->state == VFR_MULTI_POINT_ERROR) {
        return VFR_MultiPointSetFatalError(handle, VFR_SINGLE_POINT_INVALID_STATE, 1, output);
    }

    return handle->state;
}

static VFR_MultiPointState VFR_MultiPointFinishSuccessfulPoint(
    VFR_MultiPointHandle* handle, VFR_SinglePointStatus point_status,
    const VFR_SinglePointResult* point_result, VFR_MultiPointOutput* output) {
    VFR_MultiPointResult* result = &handle->config.results[handle->current_index];

    result->synchronous_component = point_result->synchronous_component;
    result->quadrature_component = point_result->quadrature_component;
    result->status = point_status;
    ++handle->processed_count;
    ++handle->successful_count;
    ++handle->current_index;
    VFR_MultiPointDisableOutput(output);

    if (handle->current_index >= handle->config.frequency_count) {
        handle->state = VFR_MULTI_POINT_COMPLETE;
        handle->status = handle->skipped_count > 0U ? VFR_MULTI_POINT_OK_WITH_SKIPPED_POINTS
                                                    : VFR_MULTI_POINT_OK;
    } else {
        handle->state = VFR_MULTI_POINT_BETWEEN_POINTS;
    }

    return handle->state;
}

VFR_MultiPointStatus VFR_MultiPointInit(VFR_MultiPointHandle* handle,
                                        const VFR_MultiPointConfig* config) {
    uint32_t index;

    if (handle == NULL) {
        return VFR_MULTI_POINT_INVALID_ARGUMENT;
    }

    *handle = (VFR_MultiPointHandle){ 0 };
    if (config == NULL) {
        return VFR_MultiPointSetInitError(handle, VFR_MULTI_POINT_INVALID_ARGUMENT);
    }

    handle->config = *config;
    if (!VFR_MultiPointIsFinitePositive(config->sample_rate_hz) ||
        !VFR_MultiPointIsFinitePositive(config->modulation_amplitude) ||
        !VFR_MultiPointIsFinitePositive(config->voltage_abs_limit) ||
        !VFR_MultiPointIsFinitePositive(config->gain_floor) || !isfinite(config->modulation_bias) ||
        config->min_samples_per_cycle == 0U || config->max_samples_per_point == 0U ||
        config->frequencies_hz == NULL || config->frequency_count == 0U ||
        config->results == NULL) {
        return VFR_MultiPointSetInitError(handle, VFR_MULTI_POINT_INVALID_ARGUMENT);
    }

    if (config->result_capacity < config->frequency_count) {
        return VFR_MultiPointSetInitError(handle, VFR_MULTI_POINT_RESULT_CAPACITY_TOO_SMALL);
    }

    for (index = 0U; index < config->frequency_count; ++index) {
        config->results[index].frequency_hz = config->frequencies_hz[index];
        config->results[index].synchronous_component = NAN;
        config->results[index].quadrature_component = NAN;
        config->results[index].status = VFR_SINGLE_POINT_INVALID_STATE;
    }

    handle->state = VFR_MULTI_POINT_READY;
    handle->status = VFR_MULTI_POINT_OK;
    return handle->status;
}

VFR_MultiPointState VFR_MultiPointTick(VFR_MultiPointHandle* handle, float actual_voltage,
                                       VFR_MultiPointOutput* output) {
    VFR_SinglePointOutput point_output = { VOLOOP_PWM_DISABLED, 0.0f };
    VFR_SinglePointState point_state;
    VFR_SinglePointStatus point_status;
    VFR_SinglePointResult point_result;

    if (handle == NULL) {
        VFR_MultiPointDisableOutput(output);
        return VFR_MULTI_POINT_ERROR;
    }

    if (output == NULL) {
        int record_current_point = handle->state != VFR_MULTI_POINT_RESET &&
                                   handle->state != VFR_MULTI_POINT_COMPLETE &&
                                   handle->state != VFR_MULTI_POINT_ERROR &&
                                   handle->current_index < handle->config.frequency_count;
        return VFR_MultiPointSetFatalError(handle, VFR_SINGLE_POINT_INVALID_ARGUMENT,
                                           record_current_point, NULL);
    }

    if (handle->state == VFR_MULTI_POINT_COMPLETE || handle->state == VFR_MULTI_POINT_ERROR) {
        VFR_MultiPointDisableOutput(output);
        return handle->state;
    }

    if (handle->state == VFR_MULTI_POINT_RESET) {
        return VFR_MultiPointSetFatalError(handle, VFR_SINGLE_POINT_INVALID_STATE, 0, output);
    }

    if (handle->state == VFR_MULTI_POINT_READY || handle->state == VFR_MULTI_POINT_BETWEEN_POINTS) {
        if (handle->current_index >= handle->config.frequency_count) {
            return VFR_MultiPointSetFatalError(handle, VFR_SINGLE_POINT_INVALID_STATE, 0, output);
        }
        return VFR_MultiPointStartCurrentPoint(handle, output);
    }

    if (handle->state != VFR_MULTI_POINT_WARMUP && handle->state != VFR_MULTI_POINT_MEASURING) {
        return VFR_MultiPointSetFatalError(handle, VFR_SINGLE_POINT_INVALID_STATE, 1, output);
    }

    point_state = VFR_SinglePointTick(&handle->point_handle, actual_voltage, &point_output);
    VFR_MultiPointCopyOutput(output, &point_output);
    if (point_state == VFR_SINGLE_POINT_ERROR) {
        point_status = VFR_SinglePointGetStatus(&handle->point_handle);
        return VFR_MultiPointSetFatalError(handle, point_status, 1, output);
    }

    if (point_state == VFR_SINGLE_POINT_COMPLETE) {
        point_status = VFR_SinglePointGetResult(&handle->point_handle, &point_result);
        if (point_status != VFR_SINGLE_POINT_OK &&
            point_status != VFR_SINGLE_POINT_OK_WITH_GAIN_FLOOR) {
            return VFR_MultiPointSetFatalError(handle, point_status, 1, output);
        }
        return VFR_MultiPointFinishSuccessfulPoint(handle, point_status, &point_result, output);
    }

    handle->state = VFR_MultiPointMapRunningState(point_state);
    if (handle->state == VFR_MULTI_POINT_ERROR) {
        return VFR_MultiPointSetFatalError(handle, VFR_SINGLE_POINT_INVALID_STATE, 1, output);
    }

    return handle->state;
}

VFR_MultiPointState VFR_MultiPointGetState(const VFR_MultiPointHandle* handle) {
    if (handle == NULL) {
        return VFR_MULTI_POINT_ERROR;
    }

    return handle->state;
}

VFR_MultiPointStatus VFR_MultiPointGetStatus(const VFR_MultiPointHandle* handle) {
    if (handle == NULL) {
        return VFR_MULTI_POINT_INVALID_ARGUMENT;
    }

    return handle->status;
}

uint32_t VFR_MultiPointGetProcessedCount(const VFR_MultiPointHandle* handle) {
    return handle == NULL ? 0U : handle->processed_count;
}

uint32_t VFR_MultiPointGetSuccessfulCount(const VFR_MultiPointHandle* handle) {
    return handle == NULL ? 0U : handle->successful_count;
}

uint32_t VFR_MultiPointGetSkippedCount(const VFR_MultiPointHandle* handle) {
    return handle == NULL ? 0U : handle->skipped_count;
}

const char* VFR_MultiPointStatusToString(VFR_MultiPointStatus status) {
    switch (status) {
        case VFR_MULTI_POINT_OK:
            return "ok";
        case VFR_MULTI_POINT_OK_WITH_SKIPPED_POINTS:
            return "ok_with_skipped_points";
        case VFR_MULTI_POINT_INVALID_ARGUMENT:
            return "invalid_argument";
        case VFR_MULTI_POINT_RESULT_CAPACITY_TOO_SMALL:
            return "result_capacity_too_small";
        case VFR_MULTI_POINT_VOLTAGE_LIMIT_EXCEEDED:
            return "voltage_limit_exceeded";
        case VFR_MULTI_POINT_NUMERIC_ERROR:
            return "numeric_error";
        case VFR_MULTI_POINT_INVALID_STATE:
            return "invalid_state";
        default:
            return "unknown";
    }
}
