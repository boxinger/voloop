#ifndef VFR_POINT_MEASURE_H
#define VFR_POINT_MEASURE_H

/**
 * @file vfr_point_measure.h
 * @brief Single-frequency black-box measurement API for VOLOOP Frequency Response tools.
 */

#include "vfr_test_subject.h"

#include <stdint.h>

typedef enum {
    VFR_POINT_OK = 0,             /**< Measurement completed without clamping or detected errors. */
    VFR_POINT_OK_WITH_GAIN_FLOOR, /**< Measurement completed, but gain was clamped to gain_floor. */
    VFR_POINT_INVALID_ARGUMENT,   /**< NULL pointers or invalid scalar configuration values. */
    VFR_POINT_INVALID_FREQUENCY,  /**< Frequency is non-positive or at/above Nyquist. */
    VFR_POINT_INSUFFICIENT_SAMPLES,  /**< Rounded samples per cycle is below min_samples_per_cycle. */
    VFR_POINT_SAMPLE_LIMIT_EXCEEDED, /**< Requested warmup + measurement samples exceed the limit. */
    VFR_POINT_NONFINITE_OUTPUT,      /**< Subject output was NaN or Inf. */
    VFR_POINT_OUTPUT_LIMIT_EXCEEDED, /**< Subject output magnitude exceeded output_abs_limit. */
    VFR_POINT_NUMERIC_ERROR,         /**< Internal measurement math produced a non-finite value. */
    VFR_POINT_SUBJECT_ERROR          /**< Reserved for future subject-level error reporting. */
} VFR_PointMeasureStatus;

/**
 * @brief Configuration for one frequency response measurement point.
 */
typedef struct {
    double sample_rate_hz;        /**< Sampling rate used to call the subject, in Hz. */
    double frequency_hz;          /**< Input sine frequency to measure, in Hz. */
    double input_amplitude;       /**< Peak amplitude of the injected sine input. */
    uint32_t warmup_cycles;       /**< Integer input cycles to run before collecting data. */
    uint32_t measure_cycles;      /**< Integer input cycles used for synchronous detection. */
    uint32_t min_samples_per_cycle; /**< Minimum acceptable rounded samples per input cycle. */
    uint32_t max_samples_per_point; /**< Maximum allowed warmup + measurement samples. */
    double output_abs_limit;      /**< Absolute output limit used to detect unstable responses. */
    double gain_floor;            /**< Minimum gain used before converting gain to dB. */
} VFR_PointMeasureConfig;

/**
 * @brief Result for one frequency response measurement point.
 */
typedef struct {
    double frequency_hz;          /**< Measured input frequency, copied from config. */
    double input_amplitude;       /**< Input peak amplitude, copied from config. */
    double output_amplitude;      /**< Measured output fundamental peak amplitude. */
    double gain_linear;           /**< Linear gain, floored to gain_floor when needed. */
    double gain_db;               /**< Gain in dB, computed as 20*log10(gain_linear). */
    double phase_deg;             /**< Wrapped phase in degrees: output phase - input phase. */
    uint32_t warmup_samples;      /**< Number of warmup samples executed before measurement. */
    uint32_t measure_samples;     /**< Number of samples used for synchronous detection. */
    uint32_t total_samples;       /**< warmup_samples + measure_samples. */
    VFR_PointMeasureStatus status; /**< Measurement status for CSV/debug output. */
} VFR_PointMeasureResult;

/**
 * @brief Measure one frequency point of a black-box input -> output subject.
 *
 * The implementation uses synchronous detection / single-frequency DFT at the
 * injected input frequency. The phase convention is output phase minus input
 * phase, so output lead is positive and output lag is negative.
 */
VFR_PointMeasureStatus VFR_MeasurePoint(VFR_TestSubject* subject,
                                        const VFR_PointMeasureConfig* config,
                                        VFR_PointMeasureResult* result);

/**
 * @brief Convert a measurement status to a stable lowercase CSV/debug string.
 *
 * Example outputs include "ok", "ok_with_gain_floor", "invalid_frequency",
 * and "output_limit_exceeded".
 */
const char* VFR_PointMeasureStatusToString(VFR_PointMeasureStatus status);

#endif /* VFR_POINT_MEASURE_H */
