#ifndef VOLOOP_DEF_H
#define VOLOOP_DEF_H

#include <stdint.h>
#include <stddef.h>
#include <math.h>

/**
 * @file voloop_def.h
 * @brief Common definitions, status codes, math helpers, and phase utilities.
 *
 * This header provides shared constants and utility APIs used by the voloop
 * control modules. It also defines weak-style customization macros for sine,
 * cosine, and debug printing so applications can override the default behavior
 * before including this header.
 */

/**
 * @defgroup VOLOOP_DEF Common Definitions
 * @ingroup VOLOOP_CORE
 * @brief Shared constants, status codes, math helpers, and phase utilities.
 *
 * This module provides the common building blocks used across voloop core
 * algorithms. It defines project-wide status codes, PWM state values,
 * single-precision math constants, Q1.31 phase conversion helpers, and
 * override hooks for sine, cosine, and debug printing.
 *
 * ## Basic usage
 *
 * Include this header directly when only the shared definitions are needed:
 *
 * @code
 * #include "voloop_def.h"
 * @endcode
 *
 * Applications may override ::VOLOOP_DEF_SIN, ::VOLOOP_DEF_COS, or
 * ::VOLOOP_DEF_PRINTF before including voloop headers.
 *
 * @{
 */

/**
 * @brief Pi constant as a single-precision floating-point value.
 */
#define VOLOOP_Pi 3.14159265358979323846f

/**
 * @brief Reciprocal of @ref VOLOOP_Pi.
 */
#define VOLOOP_Pi_Inv (1.0f / VOLOOP_Pi)

/**
 * @brief Two times pi as a single-precision floating-point value.
 */
#define VOLOOP_TwoPi (2.0f * VOLOOP_Pi)

/**
 * @brief Four times pi squared as a single-precision floating-point value.
 */
#define VOLOOP_FourPiSquared (4.0f * VOLOOP_Pi * VOLOOP_Pi)

/**
 * @brief Sine function hook used by modules that operate on Q1.31 phase values.
 *
 * Applications may define this macro before including voloop headers to replace
 * the default lookup-table implementation.
 *
 * @param x Phase in Q1.31 format.
 * @return Sine of @p x.
 */
#ifndef VOLOOP_DEF_SIN
#define VOLOOP_DEF_SIN(x) VOLOOP_DEF_SINQ31(x)
#endif

/**
 * @brief Cosine function hook used by modules that operate on Q1.31 phase values.
 *
 * Applications may define this macro before including voloop headers to replace
 * the default lookup-table implementation.
 *
 * @param x Phase in Q1.31 format.
 * @return Cosine of @p x.
 */
#ifndef VOLOOP_DEF_COS
#define VOLOOP_DEF_COS(x) VOLOOP_DEF_COSQ31(x)
#endif

/**
 * @brief Debug print hook.
 *
 * Applications may define this macro before including voloop headers to route
 * debug messages to a project-specific logging backend. The default
 * implementation discards all arguments.
 */
#ifndef VOLOOP_DEF_PRINTF
#define VOLOOP_DEF_PRINTF(...) ((void)0)
#endif

/**
 * @brief Common status codes returned by voloop APIs.
 */
typedef enum {
    VOLOOP_OK = 0x00U,    /**< Operation completed successfully. */
    VOLOOP_ERROR,         /**< Generic error. */
    VOLOOP_BAD_ALLOCATE,  /**< Allocation or storage setup failed. */
    VOLOOP_INVALID_PARAM, /**< One or more input parameters are invalid. */
    VOLOOP_INVALID_STATE, /**< Operation is not valid for the current state. */
    VOLOOP_BUSY,          /**< Resource or module is busy. */
    VOLOOP_TIMEOUT        /**< Operation timed out. */
} VOLOOP_StatusTypeDef;

/**
 * @brief Clamp a floating-point value to a closed interval.
 *
 * @param value Input value.
 * @param min Minimum allowed value.
 * @param max Maximum allowed value.
 * @return @p value limited to the range [@p min, @p max].
 */
float VOLOOP_DEF_ClampFloat(float value, float min, float max);

/**
 * @brief Convert a Q1.31 phase value to radians.
 *
 * The Q1.31 phase range maps approximately to [-pi, pi). INT32_MIN maps to
 * -pi and INT32_MAX maps close to +pi.
 *
 * @param value Phase in Q1.31 format.
 * @return Phase in radians.
 */
float VOLOOP_DEF_Q31ToRad(int32_t value);

/**
 * @brief Convert radians to a Q1.31 phase value.
 *
 * Values less than or equal to -pi saturate to INT32_MIN. Values greater than
 * or equal to +pi saturate to INT32_MAX.
 *
 * @param value Phase in radians.
 * @return Phase in Q1.31 format.
 */
int32_t VOLOOP_DEF_RadToQ31(float value);

/**
 * @brief Compute sine from a Q1.31 phase value using the built-in lookup table.
 *
 * @param phaseQ31 Phase in Q1.31 format.
 * @return Sine of @p phaseQ31.
 */
float VOLOOP_DEF_SINQ31(int32_t phaseQ31);

/**
 * @brief Compute cosine from a Q1.31 phase value using the built-in lookup table.
 *
 * @param phaseQ31 Phase in Q1.31 format.
 * @return Cosine of @p phaseQ31.
 */
float VOLOOP_DEF_COSQ31(int32_t phaseQ31);

/**
 * @brief PWM output state used by board-support and power-stage modules.
 */
typedef enum {
    VOLOOP_PWM_DISABLED = 0U, /**< PWM output is disabled. */
    VOLOOP_PWM_ENABLE,        /**< PWM output is enabled. */
} VOLOOP_DEF_PwmStateTypeDef;

/** @} */

#endif /* VOLOOP_DEF_H */
