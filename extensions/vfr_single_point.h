#ifndef VFR_SINGLE_POINT_H
#define VFR_SINGLE_POINT_H

/**
 * @file vfr_single_point.h
 * @brief Interrupt-driven single-frequency response measurement API.
 */

#include "voloop_def.h"

#include <stdint.h>

/**
 * @brief Operation and measurement status codes.
 */
typedef enum {
    VFR_SINGLE_POINT_OK = 0U,              /**< Operation or measurement completed successfully. */
    VFR_SINGLE_POINT_OK_WITH_GAIN_FLOOR,   /**< Gain was limited to the configured floor. */
    VFR_SINGLE_POINT_INVALID_ARGUMENT,     /**< A pointer or scalar parameter is invalid. */
    VFR_SINGLE_POINT_INVALID_FREQUENCY,    /**< Frequency is non-positive or at/above Nyquist. */
    VFR_SINGLE_POINT_INSUFFICIENT_SAMPLES, /**< Samples per cycle is below the configured minimum. */
    VFR_SINGLE_POINT_SAMPLE_LIMIT_EXCEEDED, /**< Measurement length exceeds its configured limit. */
    VFR_SINGLE_POINT_VOLTAGE_LIMIT_EXCEEDED, /**< A voltage sample exceeded its absolute limit. */
    VFR_SINGLE_POINT_NUMERIC_ERROR, /**< A voltage sample or internal calculation is NaN/Inf. */
    VFR_SINGLE_POINT_INVALID_STATE  /**< The requested operation is invalid in the current state. */
} VFR_SinglePointStatus;

/**
 * @brief Measurement lifecycle states.
 */
typedef enum {
    VFR_SINGLE_POINT_RESET = 0U, /**< Uninitialized or cleared handle. */
    VFR_SINGLE_POINT_READY, /**< Initialized; the first modulation suggestion has not been emitted. */
    VFR_SINGLE_POINT_WARMUP,    /**< Excitation is running without measurement accumulation. */
    VFR_SINGLE_POINT_MEASURING, /**< Voltage samples are being accumulated. */
    VFR_SINGLE_POINT_COMPLETE,  /**< All requested measurement samples have been collected. */
    VFR_SINGLE_POINT_ERROR      /**< Measurement stopped because an error was detected. */
} VFR_SinglePointState;

/**
 * @brief Configuration for one embedded frequency-response measurement point.
 *
 * The suggested modulation value is generated as:
 * @code
 * modulation_bias + modulation_amplitude * sin(phase)
 * @endcode
 *
 * The caller is responsible for selecting a bias and amplitude that are valid
 * for the external modulator. This module does not clamp the suggestion,
 * because clamping would distort the injected waveform and invalidate the
 * measured response.
 */
typedef struct {
    float sample_rate_hz;       /**< Fixed rate at which VFR_SinglePointTick is called. */
    float frequency_hz;         /**< Excitation frequency in Hz. */
    float modulation_bias;      /**< DC modulation value around which excitation is added. */
    float modulation_amplitude; /**< Peak amplitude of the injected modulation sine wave. */
    uint32_t warmup_cycles;  /**< Discarded cycles; the complete stage duration is rounded once. */
    uint32_t measure_cycles; /**< Measured cycles; the complete stage duration is rounded once. */
    uint32_t min_samples_per_cycle; /**< Minimum acceptable rounded samples per cycle. */
    uint32_t max_samples_per_point; /**< Maximum accumulated samples for this point. */
    float voltage_abs_limit;        /**< Maximum permitted absolute measured voltage. */
    float gain_floor;               /**< Minimum linear gain used when calculating gain in dB. */
} VFR_SinglePointConfig;

/**
 * @brief Final result of one frequency-response measurement point.
 */
typedef struct {
    float frequency_hz;            /**< Measured frequency, copied from the configuration. */
    float modulation_amplitude;    /**< Peak modulation perturbation used as the input. */
    float synchronous_accumulator; /**< Raw sum of voltage times the sine reference. */
    float quadrature_accumulator;  /**< Raw sum of voltage times the cosine reference. */
    float synchronous_component;   /**< Synchronous accumulator normalized by 2 / N. */
    float quadrature_component;    /**< Quadrature accumulator normalized by 2 / N. */
    float voltage_amplitude;       /**< Measured fundamental voltage peak amplitude. */
    float gain_linear;             /**< Voltage amplitude divided by modulation amplitude. */
    float gain_db;                 /**< Linear gain converted using 20 * log10(gain_linear). */
    float phase_deg;          /**< Voltage phase minus modulation phase, wrapped to [-180, 180). */
    uint32_t warmup_samples;  /**< Number of discarded settling samples. */
    uint32_t measure_samples; /**< Number of samples used for synchronous detection. */
    uint32_t total_samples;   /**< Total consumed samples, excluding the priming Tick call. */
    VFR_SinglePointStatus status; /**< Final measurement status. */
} VFR_SinglePointResult;

/**
 * @brief Power-stage output recommendation for one interrupt interval.
 */
typedef struct {
    VOLOOP_DEF_PwmStateTypeDef pwm_state; /**< Recommended PWM enable state. */
    float modulation;                     /**< Recommended modulation value when enabled. */
} VFR_SinglePointOutput;

/**
 * @brief Caller-owned configuration and runtime state.
 *
 * The handle uses only fixed-size storage and can be statically allocated. Its
 * members are public to keep the module allocation-free, but applications
 * should treat all members except @ref state and @ref status as private.
 */
typedef struct {
    VFR_SinglePointConfig config; /**< Validated configuration copy. */
    VFR_SinglePointState state;   /**< Current lifecycle state. */
    VFR_SinglePointStatus status; /**< Latest operation or terminal error status. */

    uint32_t samples_per_cycle; /**< Rounded number of interrupt samples per cycle. */
    uint32_t warmup_samples;    /**< Precomputed warmup sample count. */
    uint32_t measure_samples;   /**< Precomputed accumulation sample count. */
    uint32_t warmup_index;      /**< Number of warmup samples consumed. */
    uint32_t measure_index;     /**< Number of measurement samples accumulated. */

    uint32_t phase_accumulator; /**< Full-turn 32-bit phase accumulator. */
    uint32_t phase_step;        /**< Phase increment applied after each modulation suggestion. */
    float reference_sine;       /**< Sine associated with the latest modulation suggestion. */
    float reference_cosine;     /**< Cosine associated with the latest modulation suggestion. */
    float sine_accumulator;     /**< Voltage times sine accumulation. */
    float cosine_accumulator;   /**< Voltage times cosine accumulation. */
    float last_suggested_modulation; /**< Most recently returned modulation suggestion. */
} VFR_SinglePointHandle;

/**
 * @brief Initialize one single-frequency measurement.
 *
 * This function validates the complete configuration, precomputes sample
 * counts and phase increment, clears all accumulators, and leaves the handle
 * in VFR_SINGLE_POINT_READY. It is intended to run outside interrupt context.
 *
 * @param handle Caller-owned measurement handle.
 * @param config Measurement configuration.
 * @return VFR_SINGLE_POINT_OK on success, otherwise a validation error.
 */
VFR_SinglePointStatus VFR_SinglePointInit(VFR_SinglePointHandle* handle,
                                          const VFR_SinglePointConfig* config);

/**
 * @brief Advance the measurement by one interrupt sample.
 *
 * On the first call after initialization, @p actual_voltage is ignored and the
 * first enabled modulation suggestion is returned through @p output. On every
 * later call, @p actual_voltage must be the sample obtained while the
 * modulation value returned by the preceding call was applied. The function
 * then consumes that voltage and produces the recommendation for the next
 * sampling interval.
 *
 * Each call performs bounded, constant-storage work and contains no waiting
 * loop. When the last measurement sample is consumed, the function returns
 * VFR_SINGLE_POINT_COMPLETE and recommends VOLOOP_PWM_DISABLED with zero
 * modulation. Every error path produces the same disabled-output
 * recommendation when @p output is available.
 *
 * @param handle Initialized measurement handle.
 * @param actual_voltage Latest measured power-path voltage sample.
 * @param output Receives the PWM state and modulation recommendation for the next interval.
 * @return The state after this update. VFR_SINGLE_POINT_ERROR is returned for
 *         an invalid argument, invalid call state, non-finite sample, or
 *         voltage-limit violation.
 *
 * @note Call this function at the fixed rate configured by
 *       @ref VFR_SinglePointConfig::sample_rate_hz.
 */
VFR_SinglePointState VFR_SinglePointTick(VFR_SinglePointHandle* handle, float actual_voltage,
                                         VFR_SinglePointOutput* output);

/**
 * @brief Read the current measurement state.
 *
 * @param handle Measurement handle.
 * @return Current state, or VFR_SINGLE_POINT_ERROR if @p handle is NULL.
 */
VFR_SinglePointState VFR_SinglePointGetState(const VFR_SinglePointHandle* handle);

/**
 * @brief Read the latest operation or terminal error status.
 *
 * @param handle Measurement handle.
 * @return Current status, or VFR_SINGLE_POINT_INVALID_ARGUMENT if @p handle is NULL.
 */
VFR_SinglePointStatus VFR_SinglePointGetStatus(const VFR_SinglePointHandle* handle);

/**
 * @brief Finalize and read the completed measurement result.
 *
 * This function performs result-only calculations such as square root,
 * arctangent, and logarithm. Call it outside interrupt context after the state
 * reaches VFR_SINGLE_POINT_COMPLETE. Calling it earlier returns
 * VFR_SINGLE_POINT_INVALID_STATE.
 *
 * @param handle Completed measurement handle.
 * @param result Receives the finalized measurement result.
 * @return Final measurement status, including
 *         VFR_SINGLE_POINT_OK_WITH_GAIN_FLOOR when applicable.
 */
VFR_SinglePointStatus VFR_SinglePointGetResult(const VFR_SinglePointHandle* handle,
                                               VFR_SinglePointResult* result);

/**
 * @brief Convert a status value to a stable lowercase debug string.
 *
 * @param status Status value to convert.
 * @return Static status string.
 */
const char* VFR_SinglePointStatusToString(VFR_SinglePointStatus status);

#endif /* VFR_SINGLE_POINT_H */
