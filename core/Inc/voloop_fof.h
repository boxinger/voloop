/**
 * @file voloop_fof.h
 * @brief First-order filter (FOF) API.
 *
 * The FOF module implements a first-order discrete transfer function and
 * helper initialization modes for common continuous first-order filters.
 *
 * Runtime discrete form:
 * @code
 * H(z) = (b0 + b1 * z^-1) / (1 + a1 * z^-1)
 * @endcode
 *
 * Continuous form accepted by the generic configuration:
 * @code
 * H(s) = K * (b0 * s + b1) / (a0 * s + a1)
 * @endcode
 *
 * Continuous forms are converted to the runtime discrete form with Tustin
 * discretization.
 */
#ifndef VOLOOP_FOF_H
#define VOLOOP_FOF_H
#include "voloop_def.h"

#include <stdint.h>

/**
 * @defgroup VOLOOP_FOF First-Order Filter
 * @ingroup VOLOOP_CORE
 * @brief First-order filter and lead-lag compensator APIs.
 *
 * The FOF module implements one-pole/one-zero discrete filters and helper
 * initializers for common continuous first-order forms. Continuous modes are
 * converted to the runtime discrete form with Tustin discretization.
 *
 * ## Basic usage
 *
 * 1. Declare a ::FOF_HandleTypeDef object.
 * 2. Fill a ::FOF_InitTypeDef object and select its initialization mode.
 * 3. Call ::VOLOOP_FOF_Init.
 * 4. Call ::VOLOOP_FOF_Compute once per sample.
 *
 * ## Example
 *
 * @code
 * FOF_HandleTypeDef filter;
 * FOF_InitTypeDef init = {0};
 *
 * init.mode = FOF_LowPass;
 * init.init.LowPass.cutoffFrequency = 100.0f;
 * init.init.LowPass.triggerFrequency = 10000.0f;
 *
 * VOLOOP_FOF_Init(&filter, &init);
 * float y = VOLOOP_FOF_Compute(&filter, x);
 * @endcode
 *
 * @{
 */

/**
 * @brief FOF initialization mode.
 */
typedef enum {
    FOF_Discrete = 0U, /**< Use discrete coefficients directly. */
    FOF_Continue,      /**< Convert a generic continuous first-order form. */
    FOF_LowPass,       /**< Configure a first-order low-pass filter. */
    FOF_HighPass,      /**< Configure a first-order high-pass filter. */
    FOF_LeadLag,       /**< Configure a first-order lead-lag compensator. */
} FOF_InitModeTypeDef;

/**
 * @brief Discrete FOF coefficient initialization.
 *
 * These coefficients describe:
 * @code
 * H(z) = (b0 + b1 * z^-1) / (1 + a1 * z^-1)
 * @endcode
 */
typedef struct {
    float b0; /**< Feed-forward coefficient for the current input sample. */
    float b1; /**< Feed-forward coefficient for the previous input sample. */
    float a1; /**< Feedback coefficient for the previous output sample. */
} FOF_InitDiscreteTypeDef;

/**
 * @brief Generic continuous first-order initialization.
 *
 * This mode accepts:
 * @code
 * H(s) = K * (b0 * s + b1) / (a0 * s + a1)
 * @endcode
 */
typedef struct {
    float K;                /**< Overall gain. */
    float b0;               /**< Numerator coefficient of the s term. */
    float b1;               /**< Numerator constant coefficient. */
    float a0;               /**< Denominator coefficient of the s term. */
    float a1;               /**< Denominator constant coefficient. */
    float triggerFrequency; /**< Control loop frequency in Hz. */
} FOF_InitContinueTypeDef;

/**
 * @brief First-order low-pass filter initialization.
 */
typedef struct {
    float cutoffFrequency;  /**< Cutoff frequency in Hz. */
    float triggerFrequency; /**< Control loop frequency in Hz. */
} FOF_InitLowPassTypeDef;

/**
 * @brief First-order high-pass filter initialization.
 */
typedef struct {
    float cutoffFrequency;  /**< Cutoff frequency in Hz. */
    float triggerFrequency; /**< Control loop frequency in Hz. */
} FOF_InitHighPassTypeDef;

/**
 * @brief Lead-lag compensator initialization.
 *
 * This mode configures:
 * @code
 * H(s) = gain * (s + wz) / (s + wp)
 * @endcode
 *
 * @note @c zero and @c pole are specified in Hz, not rad/s.
 */
typedef struct {
    float zero;             /**< Zero frequency in Hz. */
    float pole;             /**< Pole frequency in Hz. */
    float gain;             /**< Compensator gain. */
    float triggerFrequency; /**< Control loop frequency in Hz. */
} FOF_InitLeadLagTypeDef;

/**
 * @brief FOF initialization container.
 *
 * Select @c mode first, then populate the matching member of @c init.
 */
typedef struct {
    FOF_InitModeTypeDef mode; /**< Initialization mode selecting the active union member. */
    union {
        FOF_InitDiscreteTypeDef Discrete; /**< Discrete coefficient initialization. */
        FOF_InitContinueTypeDef Continue; /**< Generic continuous initialization. */
        FOF_InitLowPassTypeDef LowPass;   /**< Low-pass filter initialization. */
        FOF_InitHighPassTypeDef HighPass; /**< High-pass filter initialization. */
        FOF_InitLeadLagTypeDef LeadLag;   /**< Lead-lag compensator initialization. */
    } init;                               /**< Mode-specific initialization data. */
} FOF_InitTypeDef;

/**
 * @brief FOF runtime handle.
 *
 * The handle stores normalized discrete coefficients and the previous input
 * and output samples required by the first-order difference equation.
 */
typedef struct {
    float b0; /**< Feed-forward coefficient for the current input sample. */
    float b1; /**< Feed-forward coefficient for the previous input sample. */
    float a1; /**< Feedback coefficient for the previous output sample. */

    float y1; /**< Previous output sample. */
    float x1; /**< Previous input sample. */
} FOF_HandleTypeDef;

/**
 * @brief Initialize a first-order filter and reset its runtime history.
 *
 * @param handle FOF handle to initialize.
 * @param init Initialization parameters. The active union member must match
 *             @p init->mode.
 * @return VOLOOP_OK on success, otherwise a VOLOOP error code.
 */
VOLOOP_StatusTypeDef VOLOOP_FOF_Init(FOF_HandleTypeDef* handle, const FOF_InitTypeDef* init);

/**
 * @brief Deinitialize a first-order filter handle.
 *
 * @param handle FOF handle to clear.
 * @return VOLOOP_OK on success, otherwise a VOLOOP error code.
 */
VOLOOP_StatusTypeDef VOLOOP_FOF_DeInit(FOF_HandleTypeDef* handle);

/**
 * @brief Reconfigure FOF coefficients without resetting runtime history.
 *
 * @param handle FOF handle to reconfigure.
 * @param init New initialization parameters. The active union member must match
 *             @p init->mode.
 * @return VOLOOP_OK on success, otherwise a VOLOOP error code.
 *
 * @note Call @ref VOLOOP_FOF_Reset after this function if previous samples
 *       should not be reused with the new coefficients.
 */
VOLOOP_StatusTypeDef VOLOOP_FOF_Reconfig(FOF_HandleTypeDef* handle, const FOF_InitTypeDef* init);

/**
 * @brief Reset FOF input/output history to zero.
 *
 * @param handle FOF handle to reset.
 * @return VOLOOP_OK on success, otherwise a VOLOOP error code.
 */
VOLOOP_StatusTypeDef VOLOOP_FOF_Reset(FOF_HandleTypeDef* handle);

/**
 * @brief Reset FOF input/output history to explicit values.
 *
 * @param handle FOF handle to reset.
 * @param x1 Previous input sample.
 * @param y1 Previous output sample.
 * @return VOLOOP_OK on success, otherwise a VOLOOP error code.
 */
VOLOOP_StatusTypeDef VOLOOP_FOF_ResetWithValue(FOF_HandleTypeDef* handle, float x1, float y1);

/**
 * @brief Compute one first-order filter output sample.
 *
 * @param handle FOF handle.
 * @param input Current input sample.
 * @return Current output sample. Returns 0.0f if @p handle is NULL.
 */
float VOLOOP_FOF_Compute(FOF_HandleTypeDef* handle, float input);

/** @} */

#endif /* VOLOOP_FOF_H */
