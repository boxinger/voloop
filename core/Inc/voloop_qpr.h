/**
 * @file voloop_qpr.h
 * @brief Quasi/proportional-resonant (QPR) controller API.
 *
 * The QPR module implements a second-order resonant controller for periodic
 * references or disturbances. It can be configured from already-discretized
 * coefficients, from an ideal proportional-resonant transfer function, or from
 * a non-ideal/quasi proportional-resonant transfer function.
 *
 * Ideal PR transfer function:
 * @code
 * G(s) = Kp + Kr * s / (s^2 + w0^2)
 * @endcode
 *
 * Non-ideal / quasi PR transfer function:
 * @code
 * G(s) = Kp + Kr * (2 * wc * s) / (s^2 + 2 * wc * s + w0^2)
 * @endcode
 *
 * where w0 = 2 * pi * resonantFrequency and
 * wc = 2 * pi * cutoffFrequency.
 *
 * The runtime uses the normalized discrete transfer function:
 * @code
 * H(z) = (b0 + b1 * z^-1 + b2 * z^-2) / (1 + a1 * z^-1 + a2 * z^-2)
 * @endcode
 */
#ifndef VOLOOP_QPR_H
#define VOLOOP_QPR_H
#include "voloop_def.h"

#include <stdint.h>

/**
 * @defgroup VOLOOP_QPR QPR Controller
 * @ingroup VOLOOP_CORE
 * @brief Quasi/proportional-resonant controller APIs.
 *
 * The QPR module implements a second-order resonant controller for periodic
 * references or disturbances. It can be initialized from discrete coefficients,
 * an ideal proportional-resonant controller, or a non-ideal/quasi
 * proportional-resonant controller.
 *
 * ## Basic usage
 *
 * 1. Declare a ::QPR_HandleTypeDef object.
 * 2. Fill a ::QPR_InitTypeDef object and select its initialization mode.
 * 3. Call ::VOLOOP_QPR_Init.
 * 4. Call ::VOLOOP_QPR_Compute or ::VOLOOP_QPR_ComputeBackCalculation once per
 *    sample.
 *
 * ## Example
 *
 * @code
 * QPR_HandleTypeDef qpr;
 * QPR_InitTypeDef init = {0};
 *
 * init.mode = QPR_NonIdeal;
 * init.init.NonIdeal.Kp = 0.1f;
 * init.init.NonIdeal.Kr = 20.0f;
 * init.init.NonIdeal.resonantFrequency = 50.0f;
 * init.init.NonIdeal.cutoffFrequency = 5.0f;
 * init.init.NonIdeal.triggerFrequency = 10000.0f;
 *
 * VOLOOP_QPR_Init(&qpr, &init);
 * float output = VOLOOP_QPR_Compute(&qpr, input);
 * @endcode
 *
 * @note Frequencies in QPR initialization structures are specified in Hz.
 *
 * @{
 */

/**
 * @brief QPR initialization mode.
 */
typedef enum {
    QPR_Discrete = 0U, /**< Use discrete coefficients directly. */
    QPR_Ideal,         /**< Convert an ideal PR controller with Tustin discretization. */
    QPR_NonIdeal,      /**< Convert a damped quasi-PR controller with Tustin discretization. */
} QPR_InitModeTypeDef;

/**
 * @brief QPR runtime output state.
 */
typedef enum {
    QPR_ERROR = 0U,     /**< Reserved error state, also returned for invalid handles. */
    QPR_Unsaturated,    /**< Output is inside the configured limits. */
    QPR_UpperSaturated, /**< Output was clamped to the upper limit. */
    QPR_LowerSaturated  /**< Output was clamped to the lower limit. */
} QPR_StateTypeDef;

/**
 * @brief Discrete QPR coefficient initialization.
 *
 * The coefficients describe the normalized transfer function:
 * @code
 * H(z) = (b0 + b1 * z^-1 + b2 * z^-2) / (1 + a1 * z^-1 + a2 * z^-2)
 * @endcode
 */
typedef struct {
    float b0; /**< Feed-forward coefficient for the current input sample. */
    float b1; /**< Feed-forward coefficient for the previous input sample. */
    float b2; /**< Feed-forward coefficient for the input sample two steps ago. */
    float a1; /**< Feedback coefficient for the previous output sample. */
    float a2; /**< Feedback coefficient for the output sample two steps ago. */
} QPR_InitDiscreteTypeDef;

/**
 * @brief Ideal proportional-resonant controller initialization.
 *
 * This mode represents:
 * @code
 * G(s) = Kp + Kr * s / (s^2 + w0^2)
 * @endcode
 *
 * @note Frequencies are specified in Hz, not rad/s.
 */
typedef struct {
    float Kp;                /**< Proportional gain. */
    float Kr;                /**< Resonant gain. */
    float resonantFrequency; /**< Resonant frequency in Hz. */
    float triggerFrequency;  /**< Control loop frequency in Hz. */
} QPR_InitIdealTypeDef;

/**
 * @brief Non-ideal/quasi proportional-resonant controller initialization.
 *
 * This mode represents:
 * @code
 * G(s) = Kp + Kr * (2 * wc * s) / (s^2 + 2 * wc * s + w0^2)
 * @endcode
 *
 * @note Frequencies are specified in Hz, not rad/s.
 */
typedef struct {
    float Kp;                /**< Proportional gain. */
    float Kr;                /**< Resonant gain. */
    float resonantFrequency; /**< Resonant frequency in Hz. */
    float cutoffFrequency;   /**< Resonant bandwidth / cutoff frequency in Hz. */
    float triggerFrequency;  /**< Control loop frequency in Hz. */
} QPR_InitNonIdealTypeDef;

/**
 * @brief QPR initialization container.
 *
 * Select @c mode first, then populate the matching member of @c init.
 */
typedef struct {
    QPR_InitModeTypeDef mode; /**< Initialization mode selecting the active union member. */
    union {
        QPR_InitDiscreteTypeDef Discrete; /**< Discrete coefficient initialization. */
        QPR_InitIdealTypeDef Ideal;       /**< Ideal PR initialization. */
        QPR_InitNonIdealTypeDef NonIdeal; /**< Non-ideal / quasi PR initialization. */
    } init;                               /**< Mode-specific initialization data. */
} QPR_InitTypeDef;

/**
 * @brief QPR runtime handle.
 *
 * The handle stores normalized discrete coefficients and the previous two
 * input/output samples required by the second-order difference equation.
 */
typedef struct {
    float b0; /**< Feed-forward coefficient for the current input sample. */
    float b1; /**< Feed-forward coefficient for the previous input sample. */
    float b2; /**< Feed-forward coefficient for the input sample two steps ago. */
    float a1; /**< Feedback coefficient for the previous output sample. */
    float a2; /**< Feedback coefficient for the output sample two steps ago. */

    float x1; /**< Previous input sample. */
    float x2; /**< Input sample two steps ago. */
    float y1; /**< Previous output sample. */
    float y2; /**< Output sample two steps ago. */

    QPR_StateTypeDef State; /**< Last output saturation state. */
} QPR_HandleTypeDef;

/**
 * @brief Initialize a QPR controller and reset its runtime history.
 *
 * @param handle QPR handle to initialize.
 * @param init Initialization parameters. The active union member must match
 *             @p init->mode.
 * @return VOLOOP_OK on success, otherwise a VOLOOP error code.
 */
VOLOOP_StatusTypeDef VOLOOP_QPR_Init(QPR_HandleTypeDef* handle, const QPR_InitTypeDef* init);

/**
 * @brief Deinitialize a QPR controller handle.
 *
 * @param handle QPR handle to clear.
 * @return VOLOOP_OK on success, otherwise a VOLOOP error code.
 */
VOLOOP_StatusTypeDef VOLOOP_QPR_DeInit(QPR_HandleTypeDef* handle);

/**
 * @brief Reconfigure QPR coefficients without resetting runtime history.
 *
 * @param handle QPR handle to reconfigure.
 * @param init New initialization parameters. The active union member must match
 *             @p init->mode.
 * @return VOLOOP_OK on success, otherwise a VOLOOP error code.
 *
 * @note Call @ref VOLOOP_QPR_Reset after this function if previous samples
 *       should not be reused with the new coefficients.
 */
VOLOOP_StatusTypeDef VOLOOP_QPR_Reconfig(QPR_HandleTypeDef* handle, const QPR_InitTypeDef* init);

/**
 * @brief Reset QPR input/output history to zero.
 *
 * @param handle QPR handle to reset.
 * @return VOLOOP_OK on success, otherwise a VOLOOP error code.
 */
VOLOOP_StatusTypeDef VOLOOP_QPR_Reset(QPR_HandleTypeDef* handle);

/**
 * @brief Reset QPR input/output history to explicit values.
 *
 * @param handle QPR handle to reset.
 * @param x1 Previous input sample.
 * @param x2 Input sample two steps ago.
 * @param y1 Previous output sample.
 * @param y2 Output sample two steps ago.
 * @return VOLOOP_OK on success, otherwise a VOLOOP error code.
 */
VOLOOP_StatusTypeDef VOLOOP_QPR_ResetWithValue(QPR_HandleTypeDef* handle, float x1, float x2,
                                               float y1, float y2);

/**
 * @brief Get the last QPR output saturation state.
 *
 * @param handle QPR handle.
 * @return Last QPR state, or QPR_ERROR if @p handle is NULL.
 */
QPR_StateTypeDef VOLOOP_QPR_GetState(QPR_HandleTypeDef* handle);

/**
 * @brief Compute one QPR output sample without output limiting.
 *
 * @param handle QPR handle.
 * @param input Current input sample.
 * @return Current output sample. Returns 0.0f if @p handle is NULL.
 */
float VOLOOP_QPR_Compute(QPR_HandleTypeDef* handle, float input);

/**
 * @brief Compute one QPR output sample with back-calculation anti-windup.
 *
 * The raw controller output is clamped to [@p outputMin, @p outputMax]. The
 * recursive output history is updated with a back-calculated value so the
 * internal state tracks saturation instead of accumulating unchecked output.
 *
 * @param handle QPR handle.
 * @param input Current input sample.
 * @param outputMin Minimum output limit.
 * @param outputMax Maximum output limit.
 * @param Kb Back-calculation gain. This value should be greater than 0.
 * @return Saturated output sample. Returns 0.0f if @p handle is NULL.
 */
float VOLOOP_QPR_ComputeBackCalculation(QPR_HandleTypeDef* handle, float input, float outputMin,
                                        float outputMax, float Kb);

/** @} */

#endif /* VOLOOP_QPR_H */
