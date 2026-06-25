/**
 * @file voloop_nco.h
 * @brief Numerically controlled oscillator (NCO) API.
 *
 * The NCO module maintains a 32-bit phase accumulator and exposes the phase as
 * radians, Q1.31 phase, sine, and cosine. The Q1.31 phase range matches the
 * STM32 CORDIC phase convention: [-pi, pi).
 */
#ifndef VOLOOP_NCO_H
#define VOLOOP_NCO_H

#include "voloop_def.h"
#include <stdint.h>

/**
 * @defgroup VOLOOP_NCO Numerically Controlled Oscillator
 * @ingroup VOLOOP_CORE
 * @brief Phase accumulator and sine/cosine generation APIs.
 *
 * The NCO module maintains a Q1.31 phase accumulator and exposes the current
 * phase as radians, Q1.31 value, sine, and cosine. It is typically updated
 * from a fixed-frequency timer or control-loop interrupt.
 *
 * ## Basic usage
 *
 * 1. Declare a ::NCO_HandleTypeDef object.
 * 2. Fill a ::NCO_InitTypeDef object with trigger frequency, initial frequency,
 *    and initial phase.
 * 3. Call ::VOLOOP_NCO_Init and ::VOLOOP_NCO_Start.
 * 4. Call ::VOLOOP_NCO_Sync once per trigger event.
 * 5. Read phase, sine, or cosine with the getter APIs.
 *
 * ## Example
 *
 * @code
 * NCO_HandleTypeDef nco;
 * NCO_InitTypeDef init = {
 *     .triggerFrequency = 10000U,
 *     .initialFrequency = 50.0f,
 *     .initialRad = 0.0f,
 * };
 *
 * VOLOOP_NCO_Init(&nco, &init);
 * VOLOOP_NCO_Start(&nco);
 * VOLOOP_NCO_Sync(&nco);
 *
 * float sine = VOLOOP_NCO_GetSine(&nco);
 * @endcode
 *
 * @note The output frequency must be greater than 0 and less than the trigger
 * frequency.
 *
 * @{
 */

/**
 * @brief NCO initialization parameters.
 */
typedef struct {
    uint32_t triggerFrequency; /**< NCO update frequency in Hz. */
    float initialFrequency;    /**< Initial output frequency in Hz. */
    float initialRad;          /**< Initial phase in radians, in the range [-pi, pi). */
} NCO_InitTypeDef;

/**
 * @brief NCO runtime state.
 */
typedef enum {
    NCO_ERROR = 0U, /**< Error state, also returned for invalid handles. */
    NCO_STOPPED,    /**< NCO is initialized but phase updates are stopped. */
    NCO_RUNNING     /**< NCO phase updates are enabled. */
} NCO_StateTypeDef;

/**
 * @brief NCO runtime handle.
 *
 * The handle stores initialization parameters, runtime state, current
 * frequency, phase accumulator, and precomputed phase increment.
 */
typedef struct {
    NCO_InitTypeDef Init;      /**< Copy of the initialization parameters. */
    NCO_StateTypeDef State;    /**< Current NCO state. */
    float Frequency;           /**< Current output frequency in Hz. */
    float TriggerFrequencyInv; /**< Reciprocal of the update frequency. */
    int32_t PhaseQ31;          /**< Current phase in Q1.31 format. */
    uint32_t PhaseStepQ31;     /**< Phase increment added on each sync step. */
} NCO_HandleTypeDef;

/**
 * @brief Initialize an NCO handle.
 *
 * @param handle NCO handle to initialize.
 * @param init Initialization parameters.
 * @return VOLOOP_OK on success, otherwise a VOLOOP error code.
 *
 * @note @c init->initialFrequency must be greater than 0 and less than
 *       @c init->triggerFrequency. @c init->initialRad must be in [-pi, pi).
 */
VOLOOP_StatusTypeDef VOLOOP_NCO_Init(NCO_HandleTypeDef* handle, const NCO_InitTypeDef* init);

/**
 * @brief Deinitialize an NCO handle.
 *
 * @param handle NCO handle to clear.
 * @return VOLOOP_OK on success, otherwise a VOLOOP error code.
 */
VOLOOP_StatusTypeDef VOLOOP_NCO_DeInit(NCO_HandleTypeDef* handle);

/**
 * @brief Start NCO phase updates.
 *
 * @param handle NCO handle.
 * @return VOLOOP_OK on success, otherwise a VOLOOP error code.
 */
VOLOOP_StatusTypeDef VOLOOP_NCO_Start(NCO_HandleTypeDef* handle);

/**
 * @brief Stop NCO phase updates.
 *
 * @param handle NCO handle.
 * @return VOLOOP_OK on success, otherwise a VOLOOP error code.
 */
VOLOOP_StatusTypeDef VOLOOP_NCO_Stop(NCO_HandleTypeDef* handle);

/**
 * @brief Get the current NCO state.
 *
 * @param handle NCO handle.
 * @return Current NCO state, or NCO_ERROR if @p handle is NULL.
 */
NCO_StateTypeDef VOLOOP_NCO_GetState(NCO_HandleTypeDef* handle);

/**
 * @brief Clear the NCO error state.
 *
 * @param handle NCO handle.
 * @return VOLOOP_OK on success, otherwise a VOLOOP error code.
 *
 * @note This function is valid only when the handle is currently in NCO_ERROR.
 */
VOLOOP_StatusTypeDef VOLOOP_NCO_ClearFaultCode(NCO_HandleTypeDef* handle);

/**
 * @brief Set the NCO output frequency.
 *
 * @param handle NCO handle.
 * @param frequency New output frequency in Hz.
 * @return VOLOOP_OK on success, otherwise a VOLOOP error code.
 *
 * @note @p frequency must be greater than 0 and less than the trigger frequency.
 */
VOLOOP_StatusTypeDef VOLOOP_NCO_SetFrequency(NCO_HandleTypeDef* handle, float frequency);

/**
 * @brief Get the current NCO output frequency.
 *
 * @param handle NCO handle.
 * @return Current frequency in Hz, or 0.0f if @p handle is NULL.
 */
float VOLOOP_NCO_GetFrequency(NCO_HandleTypeDef* handle);

/**
 * @brief Set the current NCO phase in radians.
 *
 * @param handle NCO handle.
 * @param rad New phase in radians, in the range [-pi, pi).
 * @return VOLOOP_OK on success, otherwise a VOLOOP error code.
 *
 * @note If @p rad is outside the valid range, the handle enters NCO_ERROR.
 */
VOLOOP_StatusTypeDef VOLOOP_NCO_SetRad(NCO_HandleTypeDef* handle, float rad);

/**
 * @brief Get the current NCO phase in radians.
 *
 * @param handle NCO handle.
 * @return Current phase in radians, or 0.0f if @p handle is NULL.
 */
float VOLOOP_NCO_GetRad(NCO_HandleTypeDef* handle);

/**
 * @brief Get the current NCO phase in Q1.31 format.
 *
 * @param handle NCO handle.
 * @return Current Q1.31 phase, or 0 if @p handle is NULL.
 */
int32_t VOLOOP_NCO_GetPhaseQ31(NCO_HandleTypeDef* handle);

/**
 * @brief Get the address of the current Q1.31 phase value.
 *
 * This is useful when a peripheral or DSP routine needs to read the current
 * phase value by pointer.
 *
 * @param handle NCO handle.
 * @return Address of the Q1.31 phase value, or NULL if @p handle is NULL.
 */
const int32_t* VOLOOP_NCO_GetPhaseQ31Address(NCO_HandleTypeDef* handle);

/**
 * @brief Get the sine of the current NCO phase.
 *
 * @param handle NCO handle.
 * @return Sine of the current phase, or 0.0f if @p handle is NULL.
 */
float VOLOOP_NCO_GetSine(NCO_HandleTypeDef* handle);

/**
 * @brief Get the cosine of the current NCO phase.
 *
 * @param handle NCO handle.
 * @return Cosine of the current phase, or 0.0f if @p handle is NULL.
 */
float VOLOOP_NCO_GetCosine(NCO_HandleTypeDef* handle);

/**
 * @brief Advance the NCO phase by one update step.
 *
 * @param handle NCO handle.
 * @return VOLOOP_OK on success, otherwise a VOLOOP error code.
 *
 * @note This function is valid only while the NCO is running.
 */
VOLOOP_StatusTypeDef VOLOOP_NCO_Sync(NCO_HandleTypeDef* handle);

/** @} */

#endif /* VOLOOP_NCO_H */
