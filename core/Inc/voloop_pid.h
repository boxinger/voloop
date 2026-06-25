/**
 * @file voloop_pid.h
 * @brief Proportional-integral-derivative (PID) controller API.
 *
 * The PID module can be initialized from already-discretized gains, from
 * continuous PID gains, or from zero-placement forms commonly used in control
 * loop compensation.
 *
 * One-zero form:
 * @code
 * Gc(s) = Kp + Ki / s
 *       = K * (s + wz) / s
 * @endcode
 *
 * Two-zero form:
 * @code
 * Gc(s) = Kd * s + Kp + Ki / s
 *       = K * (s + wz1) * (s + wz2) / s
 * @endcode
 *
 * Frequencies used by zero-placement initialization are specified in Hz.
 */
#ifndef VOLOOP_PID_H
#define VOLOOP_PID_H

#include "voloop_def.h"
#include <stdint.h>

/**
 * @defgroup VOLOOP_PID PID Controller
 * @ingroup VOLOOP_CORE
 * @brief PID controller initialization, state management, and runtime execution.
 *
 * The PID module provides multiple initialization modes for discrete and
 * continuous control-loop designs. It supports plain PID execution, output
 * limiting with conditional integration, and back-calculation anti-windup.
 *
 * ## Basic usage
 *
 * 1. Declare a ::PID_HandleTypeDef object.
 * 2. Fill a ::PID_InitTypeDef object and select its initialization mode.
 * 3. Call ::VOLOOP_PID_Init.
 * 4. Call ::VOLOOP_PID_Compute, ::VOLOOP_PID_ComputeConditional, or
 *    ::VOLOOP_PID_ComputeBackCalculation once per control-loop sample.
 *
 * ## Example
 *
 * @code
 * PID_HandleTypeDef pid;
 * PID_InitTypeDef init = {0};
 *
 * init.mode = PID_Discrete;
 * init.init.Discrete.KpDiscrete = 1.0f;
 * init.init.Discrete.KiDiscrete = 0.01f;
 * init.init.Discrete.KdDiscrete = 0.0f;
 *
 * VOLOOP_PID_Init(&pid, &init);
 *
 * float output = VOLOOP_PID_ComputeConditional(&pid, setpoint, measurement,
 *                                              0.0f, 1.0f);
 * @endcode
 *
 * @note PID compute APIs should be called at a fixed sampling interval.
 *
 * @{
 */

/**
 * @brief Discrete PID gain initialization.
 *
 * These gains are used directly by the runtime difference equation. The
 * integral term accumulates raw error samples and the derivative term uses the
 * difference between the current and previous error samples.
 */
typedef struct {
    float KpDiscrete; /**< Discrete proportional gain. */
    float KiDiscrete; /**< Discrete integral gain. */
    float KdDiscrete; /**< Discrete derivative gain. */
} PID_InitDiscreteTypeDef;

/**
 * @brief Continuous PID gain initialization.
 *
 * Continuous gains are converted internally using the control-loop trigger
 * frequency:
 * @code
 * KpDiscrete = Kp
 * KiDiscrete = Ki / triggerFrequency
 * KdDiscrete = Kd * triggerFrequency
 * @endcode
 */
typedef struct {
    float Kp;                  /**< Continuous proportional gain. */
    float Ki;                  /**< Continuous integral gain. */
    float Kd;                  /**< Continuous derivative gain. */
    uint32_t triggerFrequency; /**< Control loop frequency in Hz. */
} PID_InitContinueTypeDef;

/**
 * @brief One-zero compensator initialization.
 *
 * This mode configures:
 * @code
 * Gc(s) = K * (s + wz) / s
 * @endcode
 *
 * @note @ref zero is specified in Hz, not rad/s.
 */
typedef struct {
    float gain;                /**< Compensator gain K. */
    float zero;                /**< Zero frequency in Hz. */
    uint32_t triggerFrequency; /**< Control loop frequency in Hz. */
} PID_InitOneZeroTypeDef;

/**
 * @brief Two-zero compensator initialization.
 *
 * This mode configures:
 * @code
 * Gc(s) = K * (s + wz1) * (s + wz2) / s
 * @endcode
 *
 * @note @ref zero1 and @ref zero2 are specified in Hz, not rad/s.
 */
typedef struct {
    float gain;                /**< Compensator gain K. */
    float zero1;               /**< First zero frequency in Hz. */
    float zero2;               /**< Second zero frequency in Hz. */
    uint32_t triggerFrequency; /**< Control loop frequency in Hz. */
} PID_InitTwoZeroTypeDef;

/**
 * @brief PID runtime output state.
 */
typedef enum {
    PID_ERROR = 0U,     /**< Reserved error state, also returned for invalid handles. */
    PID_UnSaturated,    /**< Output is inside the configured limits. */
    PID_UpperSaturated, /**< Output was clamped to the upper limit. */
    PID_LowerSaturated  /**< Output was clamped to the lower limit. */
} PID_StateTypeDef;

/**
 * @brief PID runtime handle.
 *
 * The handle stores the discrete gains and the state needed by the integral
 * and derivative terms.
 */
typedef struct {
    float KpDiscrete;       /**< Discrete proportional gain. */
    float KiDiscrete;       /**< Discrete integral gain. */
    float KdDiscrete;       /**< Discrete derivative gain. */
    float Integral;         /**< Accumulated error used by the integral term. */
    float PreviousError;    /**< Previous error sample used by the derivative term. */
    PID_StateTypeDef State; /**< Last output saturation state. */
} PID_HandleTypeDef;

/**
 * @brief PID initialization mode.
 */
typedef enum {
    PID_Discrete = 0U, /**< Use discrete PID gains directly. */
    PID_Continue,      /**< Convert continuous PID gains to discrete gains. */
    PID_OneZero,       /**< Convert a one-zero compensator to PID gains. */
    PID_TwoZero        /**< Convert a two-zero compensator to PID gains. */
} PID_InitModeTypeDef;

/**
 * @brief PID initialization container.
 *
 * Select @c mode first, then populate the matching member of @c init.
 */
typedef struct {
    PID_InitModeTypeDef mode; /**< Initialization mode selecting the active union member. */
    union {
        PID_InitDiscreteTypeDef Discrete; /**< Discrete gain initialization. */
        PID_InitContinueTypeDef Continue; /**< Continuous gain initialization. */
        PID_InitOneZeroTypeDef OneZero;   /**< One-zero compensator initialization. */
        PID_InitTwoZeroTypeDef TwoZero;   /**< Two-zero compensator initialization. */
    } init;                               /**< Mode-specific initialization data. */
} PID_InitTypeDef;

/**
 * @brief Initialize a PID controller and reset its runtime state.
 *
 * @param handle PID handle to initialize.
 * @param init Initialization parameters. The active union member must match
 *             @p init->mode.
 * @return VOLOOP_OK on success, otherwise a VOLOOP error code.
 */
VOLOOP_StatusTypeDef VOLOOP_PID_Init(PID_HandleTypeDef* handle, const PID_InitTypeDef* init);

/**
 * @brief Deinitialize a PID controller handle.
 *
 * @param handle PID handle to clear.
 * @return VOLOOP_OK on success, otherwise a VOLOOP error code.
 */
VOLOOP_StatusTypeDef VOLOOP_PID_DeInit(PID_HandleTypeDef* handle);

/**
 * @brief Reset PID integral and derivative history.
 *
 * This function preserves the configured discrete gains.
 *
 * @param handle PID handle to reset.
 * @return VOLOOP_OK on success, otherwise a VOLOOP error code.
 */
VOLOOP_StatusTypeDef VOLOOP_PID_Reset(PID_HandleTypeDef* handle);

/**
 * @brief Set the accumulated integral state.
 *
 * @param handle PID handle.
 * @param integral New accumulated integral value.
 * @return VOLOOP_OK on success, otherwise a VOLOOP error code.
 */
VOLOOP_StatusTypeDef VOLOOP_PID_SetIntegral(PID_HandleTypeDef* handle, float integral);

/**
 * @brief Set the previous error state.
 *
 * @param handle PID handle.
 * @param previousError New previous error value.
 * @return VOLOOP_OK on success, otherwise a VOLOOP error code.
 */
VOLOOP_StatusTypeDef VOLOOP_PID_SetPreviousError(PID_HandleTypeDef* handle, float previousError);

/**
 * @brief Get the last PID output saturation state.
 *
 * @param handle PID handle.
 * @return Last PID state, or PID_ERROR if @p handle is NULL.
 */
PID_StateTypeDef VOLOOP_PID_GetState(PID_HandleTypeDef* handle);

/**
 * @brief Compute one PID output sample without output limiting.
 *
 * @param handle PID handle.
 * @param setpoint Target value.
 * @param measurement Measured feedback value.
 * @return Controller output. Returns 0.0f if @p handle is NULL.
 */
float VOLOOP_PID_Compute(PID_HandleTypeDef* handle, float setpoint, float measurement);

/**
 * @brief Compute one PID output sample with conditional integration anti-windup.
 *
 * The returned output is clamped to [@p outputMin, @p outputMax]. The integral
 * state is held when the raw output is saturated and the current error would
 * drive the output further into saturation.
 *
 * @param handle PID handle.
 * @param setpoint Target value.
 * @param measurement Measured feedback value.
 * @param outputMin Minimum output limit.
 * @param outputMax Maximum output limit.
 * @return Saturated controller output. Returns 0.0f if @p handle is NULL.
 */
float VOLOOP_PID_ComputeConditional(PID_HandleTypeDef* handle, float setpoint, float measurement,
                                    float outputMin, float outputMax);

/**
 * @brief Compute one PID output sample with back-calculation anti-windup.
 *
 * The raw controller output is clamped to [@p outputMin, @p outputMax]. When
 * clamping occurs, the integral state is corrected by the back-calculation
 * term so it tracks the saturated output.
 *
 * @param handle PID handle.
 * @param setpoint Target value.
 * @param measurement Measured feedback value.
 * @param outputMin Minimum output limit.
 * @param outputMax Maximum output limit.
 * @param Kb Back-calculation gain. This value should be greater than 0.
 * @return Saturated controller output. Returns 0.0f if @p handle is NULL.
 */
float VOLOOP_PID_ComputeBackCalculation(PID_HandleTypeDef* handle, float setpoint,
                                        float measurement, float outputMin, float outputMax,
                                        float Kb);

/** @} */

#endif /* VOLOOP_PID_H */
