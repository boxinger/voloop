#ifndef VFR_MULTI_POINT_H
#define VFR_MULTI_POINT_H

/**
 * @file vfr_multi_point.h
 * @brief Interrupt-driven multi-frequency response measurement API.
 */

#include "vfr_single_point.h"

#include <stdint.h>

/**
 * @brief Overall multi-frequency scan status codes.
 */
typedef enum {
    VFR_MULTI_POINT_OK = 0U, /**< The scan is valid and no frequency has been skipped. */
    VFR_MULTI_POINT_OK_WITH_SKIPPED_POINTS, /**< The scan completed or is running with skipped points. */
    VFR_MULTI_POINT_INVALID_ARGUMENT,       /**< A pointer or common scalar parameter is invalid. */
    VFR_MULTI_POINT_RESULT_CAPACITY_TOO_SMALL, /**< The result buffer cannot hold every frequency. */
    VFR_MULTI_POINT_VOLTAGE_LIMIT_EXCEEDED,    /**< A running point exceeded the voltage limit. */
    VFR_MULTI_POINT_NUMERIC_ERROR, /**< A running point produced invalid numeric data. */
    VFR_MULTI_POINT_INVALID_STATE  /**< The scan or underlying point entered an invalid state. */
} VFR_MultiPointStatus;

/**
 * @brief Multi-frequency scan lifecycle states.
 */
typedef enum {
    VFR_MULTI_POINT_RESET = 0U,     /**< Uninitialized or cleared handle. */
    VFR_MULTI_POINT_READY,          /**< Initialized; the first point has not started. */
    VFR_MULTI_POINT_WARMUP,         /**< The current point is settling. */
    VFR_MULTI_POINT_MEASURING,      /**< The current point is accumulating samples. */
    VFR_MULTI_POINT_BETWEEN_POINTS, /**< PWM is disabled between two frequency points. */
    VFR_MULTI_POINT_COMPLETE,       /**< Every requested frequency has been processed. */
    VFR_MULTI_POINT_ERROR           /**< The scan stopped because of a fatal error. */
} VFR_MultiPointState;

/**
 * @brief Stored result for one requested frequency.
 *
 * Results use the same index as the configured frequency list. A skipped or
 * unprocessed point retains NaN components. Before a point is processed its
 * status is VFR_SINGLE_POINT_INVALID_STATE.
 */
typedef struct {
    float frequency_hz;           /**< Requested frequency for this result slot. */
    float synchronous_component;  /**< Normalized in-phase voltage component. */
    float quadrature_component;   /**< Normalized quadrature voltage component. */
    VFR_SinglePointStatus status; /**< Direct status produced by the single-point module. */
} VFR_MultiPointResult;

/**
 * @brief Power-stage output recommendation for one interrupt interval.
 */
typedef struct {
    VOLOOP_DEF_PwmStateTypeDef pwm_state; /**< Recommended PWM enable state. */
    float modulation;                     /**< Recommended modulation value when enabled. */
} VFR_MultiPointOutput;

/**
 * @brief Configuration for one multi-frequency scan.
 *
 * @p frequencies_hz and @p results are caller-owned and must remain valid and
 * unmodified until the scan reaches VFR_MULTI_POINT_COMPLETE or
 * VFR_MULTI_POINT_ERROR. The module does not allocate dynamic memory.
 */
typedef struct {
    float sample_rate_hz;           /**< Fixed rate at which VFR_MultiPointTick is called. */
    float modulation_bias;          /**< DC modulation value around which excitation is added. */
    float modulation_amplitude;     /**< Peak amplitude of the injected modulation sine wave. */
    uint32_t min_samples_per_cycle; /**< Minimum acceptable rounded samples per cycle. */
    uint32_t max_samples_per_point; /**< Maximum accumulated measurement samples per point. */
    float voltage_abs_limit;        /**< Maximum permitted absolute measured voltage. */
    float gain_floor;               /**< Gain floor passed to the single-point calculation. */

    const float* frequencies_hz;   /**< Ordered requested frequency list. */
    uint32_t frequency_count;      /**< Number of entries in @p frequencies_hz. */
    VFR_MultiPointResult* results; /**< Result buffer indexed like @p frequencies_hz. */
    uint32_t result_capacity;      /**< Number of available entries in @p results. */
} VFR_MultiPointConfig;

/**
 * @brief Caller-owned configuration and runtime state.
 *
 * The handle uses fixed-size storage. Applications should treat all members
 * except @ref state and @ref status as private.
 */
typedef struct {
    VFR_MultiPointConfig config; /**< Validated configuration copy. */
    VFR_MultiPointState state;   /**< Current scan state. */
    VFR_MultiPointStatus status; /**< Overall scan status. */

    VFR_SinglePointHandle point_handle; /**< Reused single-frequency measurement handle. */
    uint32_t current_index;             /**< Index of the point being prepared or measured. */
    uint32_t processed_count;  /**< Terminal result slots, including a fatal current point. */
    uint32_t successful_count; /**< Successfully measured points. */
    uint32_t skipped_count;    /**< Points skipped before excitation started. */
} VFR_MultiPointHandle;

/**
 * @brief Initialize a multi-frequency response scan.
 *
 * This validates common configuration, initializes every result slot to its
 * requested frequency plus NaN components, and leaves the handle in
 * VFR_MULTI_POINT_READY. Individual frequencies are prepared when their Tick
 * is reached so that an invalid point can be skipped independently.
 *
 * @param handle Caller-owned scan handle.
 * @param config Scan configuration and caller-owned buffers.
 * @return VFR_MULTI_POINT_OK on success, otherwise an initialization error.
 */
VFR_MultiPointStatus VFR_MultiPointInit(VFR_MultiPointHandle* handle,
                                        const VFR_MultiPointConfig* config);

/**
 * @brief Advance the scan by one interrupt sample.
 *
 * A completed point returns a disabled output for the next interval. The
 * following call prepares and starts the next valid point. Each skipped point
 * consumes one call with a disabled output. Fatal runtime errors permanently
 * disable the output and place the scan in VFR_MULTI_POINT_ERROR.
 *
 * @param handle Initialized scan handle.
 * @param actual_voltage Voltage measured while the preceding recommendation was applied.
 * @param output Receives the recommendation for the next interrupt interval.
 * @return Scan state after this update.
 */
VFR_MultiPointState VFR_MultiPointTick(VFR_MultiPointHandle* handle, float actual_voltage,
                                       VFR_MultiPointOutput* output);

/** @brief Read the current scan state. */
VFR_MultiPointState VFR_MultiPointGetState(const VFR_MultiPointHandle* handle);

/** @brief Read the current overall scan status. */
VFR_MultiPointStatus VFR_MultiPointGetStatus(const VFR_MultiPointHandle* handle);

/** @brief Read the number of terminal result slots. */
uint32_t VFR_MultiPointGetProcessedCount(const VFR_MultiPointHandle* handle);

/** @brief Read the number of successfully measured points. */
uint32_t VFR_MultiPointGetSuccessfulCount(const VFR_MultiPointHandle* handle);

/** @brief Read the number of points skipped before excitation. */
uint32_t VFR_MultiPointGetSkippedCount(const VFR_MultiPointHandle* handle);

/** @brief Convert an overall status value to a stable lowercase debug string. */
const char* VFR_MultiPointStatusToString(VFR_MultiPointStatus status);

#endif /* VFR_MULTI_POINT_H */
