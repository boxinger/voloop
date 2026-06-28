/**
 * @file voloop_pll.h
 * @brief Phase-locked loop (PLL) API.
 */
#ifndef VOLOOP_PLL_H
#define VOLOOP_PLL_H

#include "voloop_pid.h"
#include "voloop_nco.h"
#include "voloop_def.h"

/**
 * @defgroup VOLOOP_PLL Phase-Locked Loop
 * @ingroup VOLOOP_CORE
 * @brief Single-phase PLL control APIs built from PID and NCO modules.
 *
 * The PLL module combines a loop filter and numerically controlled oscillator
 * to estimate phase and frequency from a measured input voltage. It exposes
 * lifecycle control, lock-state queries, and per-sample synchronization APIs.
 *
 * ## Basic usage
 *
 * 1. Prepare ::PID_InitTypeDef and ::NCO_InitTypeDef objects.
 * 2. Fill a ::PLL_InitTypeDef object with the loop filter, NCO, and trigger
 *    frequency.
 * 3. Call ::VOLOOP_PLL_Init and ::VOLOOP_PLL_Start.
 * 4. Call ::VOLOOP_PLL_Sync once per control-loop sample.
 * 5. Read phase, frequency, and lock state with the getter APIs.
 *
 * @{
 */

typedef struct {
    float InputVoltage; /**< Measured PLL input voltage sample. */
} PLL_InputTypeDef;

/**
 * @brief PLL initialization parameters.
 */
typedef struct {
    const PID_InitTypeDef* LoopFilterInit; /**< Loop-filter PID initialization. */
    const NCO_InitTypeDef* NCOInit;        /**< NCO initialization used by the phase generator. */
    const float triggerFrequency;          /**< Control loop frequency in Hz. */
} PLL_InitTypeDef;

/**
 * @brief PLL runtime state.
 */
typedef enum {
    PLL_RESET = 0U, /**< Reset or uninitialized state. */
    PLL_ERROR,      /**< Error state caused by invalid operation or child-module failure. */
    PLL_STOPPED,    /**< PLL is initialized but not running. */
    PLL_RUNNING     /**< PLL is running and accepts sync updates. */
} PLL_StateTypeDef;

/**
 * @brief PLL lock-detection state.
 */
typedef enum {
    PLL_UNLOCKED = 0U, /**< PLL is not locked to the input signal. */
    PLL_LOCKED,        /**< PLL is locked to the input signal. */
    PLL_NO_SIGNAL      /**< Input signal amplitude is too small for lock detection. */
} PLL_LockStateTypeDef;

/**
 * @brief PLL runtime handle.
 *
 * The handle owns the loop-filter PID controller, the NCO phase generator,
 * SOGI phase-detector state, amplitude normalization state, lock-detection
 * counters, and latest estimated phase/frequency outputs.
 */
typedef struct {
    PID_HandleTypeDef LoopFilter; /**< PID loop filter used to correct NCO frequency. */
    NCO_HandleTypeDef NCO;        /**< NCO used to generate phase and quadrature references. */
    float NominalFrequency;       /**< Nominal input frequency in Hz. */
    float triggerFrequency;       /**< Control loop frequency in Hz. */
    float triggerFrequencyInv;    /**< Reciprocal of @ref triggerFrequency. */

    float sogiCenterFrequency; /**< Last NCO frequency used to calculate SOGI coefficients. */
    float sogiSinB0;           /**< SOGI in-phase feed-forward coefficient for x[n]. */
    float sogiSinB1;           /**< SOGI in-phase feed-forward coefficient for x[n-1]. */
    float sogiSinB2;           /**< SOGI in-phase feed-forward coefficient for x[n-2]. */
    float sogiCosB0;           /**< SOGI quadrature feed-forward coefficient for x[n]. */
    float sogiCosB1;           /**< SOGI quadrature feed-forward coefficient for x[n-1]. */
    float sogiCosB2;           /**< SOGI quadrature feed-forward coefficient for x[n-2]. */
    float sogiA1;              /**< SOGI shared feedback coefficient for y[n-1]. */
    float sogiA2;              /**< SOGI shared feedback coefficient for y[n-2]. */
    float sogiX1;              /**< Previous normalized SOGI input sample. */
    float sogiX2;              /**< Second previous normalized SOGI input sample. */
    float sogiSinY1;           /**< Previous SOGI in-phase output sample. */
    float sogiSinY2;           /**< Second previous SOGI in-phase output sample. */
    float sogiCosY1;           /**< Previous SOGI quadrature output sample. */
    float sogiCosY2;           /**< Second previous SOGI quadrature output sample. */

    float dcAlpha;                /**< DC-offset tracking coefficient. */
    float squareAlpha;            /**< Mean-square tracking coefficient. */
    float dcValue;                /**< Estimated input DC offset. */
    float squareAvg;              /**< Estimated mean square of the AC input component. */
    float peakValue;              /**< Estimated input peak value. */
    float peakValueInv;           /**< Reciprocal of @ref peakValue used for normalization. */
    uint8_t InvPeakUpdateCounter; /**< Counter controlling reciprocal peak updates. */

    int32_t PhaseQ31;               /**< Latest estimated phase in Q1.31 format. */
    float Frequency;                /**< Latest estimated frequency in Hz. */
    float PreviousFrequencyCorrection; /**< Previous loop-filter frequency correction in Hz. */
    uint16_t LockCounter;           /**< Consecutive samples meeting the lock criteria. */
    uint16_t UnlockCounter;         /**< Consecutive samples failing the lock criteria. */
    PLL_LockStateTypeDef LockState; /**< Current lock-detection state. */
    PLL_StateTypeDef State;         /**< Current PLL runtime state. */
} PLL_HandleTypeDef;

/**
 * @brief Initialize a PLL handle.
 *
 * @param handle PLL handle to initialize.
 * @param init Initialization parameters.
 * @return VOLOOP_OK on success, otherwise a VOLOOP error code.
 */
VOLOOP_StatusTypeDef VOLOOP_PLL_Init(PLL_HandleTypeDef* handle, const PLL_InitTypeDef* init);

/**
 * @brief Deinitialize a PLL handle.
 *
 * @param handle PLL handle to clear.
 * @return VOLOOP_OK on success, otherwise a VOLOOP error code.
 */
VOLOOP_StatusTypeDef VOLOOP_PLL_DeInit(PLL_HandleTypeDef* handle);

/**
 * @brief Start PLL tracking.
 *
 * This starts the internal NCO, resets the loop filter and lock counters, and
 * moves the PLL to PLL_RUNNING.
 *
 * @param handle PLL handle.
 * @return VOLOOP_OK on success, otherwise a VOLOOP error code.
 */
VOLOOP_StatusTypeDef VOLOOP_PLL_Start(PLL_HandleTypeDef* handle);

/**
 * @brief Stop PLL tracking.
 *
 * @param handle PLL handle.
 * @return VOLOOP_OK on success, otherwise a VOLOOP error code.
 */
VOLOOP_StatusTypeDef VOLOOP_PLL_Stop(PLL_HandleTypeDef* handle);

/**
 * @brief Reset PLL runtime state while preserving configuration.
 *
 * This resets the loop filter, NCO frequency/phase, amplitude estimator, and
 * lock-detection counters. The PLL returns to PLL_STOPPED on success.
 *
 * @param handle PLL handle.
 * @return VOLOOP_OK on success, otherwise a VOLOOP error code.
 */
VOLOOP_StatusTypeDef VOLOOP_PLL_Reset(PLL_HandleTypeDef* handle);

/**
 * @brief Get the current PLL runtime state.
 *
 * @param handle PLL handle.
 * @return Current PLL state, or PLL_ERROR if @p handle is NULL.
 */
PLL_StateTypeDef VOLOOP_PLL_GetState(const PLL_HandleTypeDef* handle);

/**
 * @brief Get the current PLL lock-detection state.
 *
 * @param handle PLL handle.
 * @return Current lock state, or PLL_UNLOCKED if @p handle is NULL.
 */
PLL_LockStateTypeDef VOLOOP_PLL_IsLocked(const PLL_HandleTypeDef* handle);

/**
 * @brief Get the latest estimated PLL phase in Q1.31 format.
 *
 * @param handle PLL handle.
 * @return Current phase in Q1.31 format, or 0 if @p handle is NULL.
 */
int32_t VOLOOP_PLL_GetPhaseQ31(const PLL_HandleTypeDef* handle);

/**
 * @brief Get the latest estimated PLL phase in radians.
 *
 * @param handle PLL handle.
 * @return Current phase in radians in the range [-pi, pi), or 0.0f if
 *         @p handle is NULL.
 */
float VOLOOP_PLL_GetRad(const PLL_HandleTypeDef* handle);

/**
 * @brief Get the latest estimated PLL frequency.
 *
 * @param handle PLL handle.
 * @return Current estimated frequency in Hz, or 0.0f if @p handle is NULL.
 */
float VOLOOP_PLL_GetFrequency(const PLL_HandleTypeDef* handle);

/**
 * @brief Run one PLL control-loop update.
 *
 * This function updates input normalization, computes the SOGI-based phase
 * error, applies the loop filter as a frequency correction, advances the
 * internal NCO, updates the exported phase/frequency, and refreshes
 * lock-detection state.
 *
 * @param handle PLL handle.
 * @param input Measured input sample.
 * @return VOLOOP_OK on success or stopped state, otherwise a VOLOOP error code.
 *
 * @note If the PLL is PLL_STOPPED, this function returns VOLOOP_OK without
 *       updating runtime state.
 */
VOLOOP_StatusTypeDef VOLOOP_PLL_Sync(PLL_HandleTypeDef* handle, const PLL_InputTypeDef* input);

/** @} */

#endif /* VOLOOP_PLL_H */
