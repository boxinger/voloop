/**
 * @file voloop_offinv.h
 * @brief Off-grid inverter control API.
 */
#ifndef VOLOOP_OFFINV_H
#define VOLOOP_OFFINV_H

#include "voloop_def.h"
#include "voloop_qpr.h"
#include "voloop_nco.h"

/**
 * @defgroup VOLOOP_OFFINV Off-Grid Inverter
 * @ingroup VOLOOP_CORE
 * @brief Off-grid inverter voltage-control APIs built from QPR and NCO modules.
 *
 * The off-grid inverter module combines a resonant voltage controller and NCO
 * phase generator to produce PWM duty commands from measured output voltage
 * and current. It also tracks runtime state and fault conditions.
 *
 * ## Basic usage
 *
 * 1. Prepare ::QPR_InitTypeDef and ::NCO_InitTypeDef objects.
 * 2. Fill an ::OffInv_InitTypeDef object.
 * 3. Call ::VOLOOP_OffInv_Init and ::VOLOOP_OffInv_Start.
 * 4. Set the voltage target with ::VOLOOP_OffInv_SetValue.
 * 5. Call ::VOLOOP_OffInv_Sync once per control-loop sample.
 *
 * @{
 */

typedef struct {
    float OutputVoltage; /**< Measured inverter output voltage. */
    float OutputCurrent; /**< Measured inverter output current. */
} OffInv_InputTypeDef;

/**
 * @brief Off-grid inverter PWM output command.
 */
typedef struct {
    VOLOOP_DEF_PwmStateTypeDef LeftLegPwmState; /**< Left bridge leg PWM state. */
    float LeftLegDuty; /**< Left bridge leg duty command, normally in [0, OFFINV_MAX_DUTY]. */
    VOLOOP_DEF_PwmStateTypeDef RightLegPwmState; /**< Right bridge leg PWM state. */
    float RightLegDuty; /**< Right bridge leg duty command, normally in [0, OFFINV_MAX_DUTY]. */
} OffInv_OutputTypeDef;

/**
 * @brief Off-grid inverter initialization parameters.
 */
typedef struct {
    const QPR_InitTypeDef* VoltageQPRInit; /**< Voltage-loop QPR initialization. */
    const NCO_InitTypeDef* NCOInit; /**< NCO initialization used to generate voltage phase. */
    float InputVoltage;             /**< Fixed inverter input voltage used for feed-forward. */
    const float triggerFrequency;   /**< Control loop frequency in Hz. Use the same trigger
                                         frequency for this module, QPR, and NCO during init. */
} OffInv_InitTypeDef;

/**
 * @brief Off-grid inverter runtime state.
 */
typedef enum {
    OFFINV_ERROR = 0U, /**< Error state, usually caused by protection or child-module failure. */
    OFFINV_DISABLED,   /**< Inverter controller is initialized but not running. */
    OFFINV_RUNNING,    /**< Inverter controller is running and accepts sync updates. */
} OffInv_StateTypeDef;

/**
 * @brief Off-grid inverter protection fault code.
 */
typedef enum {
    OFFINV_INVALID = 0U, /**< Invalid fault code, also returned for invalid handles. */
    OFFINV_NOERROR,      /**< No active fault. */
    OFFINV_OCP,          /**< Over-current protection fault. */
    OFFINV_OVP,          /**< Over-voltage protection fault. */
    OFFINV_NCO,          /**< NCO child-module fault. */
} OffInv_FaultCodeTypeDef;

/**
 * @brief Off-grid inverter runtime handle.
 *
 * The handle owns the voltage QPR controller, the NCO phase generator, the
 * latest target voltage, fault state, and generated modulation duty.
 */
typedef struct {
    QPR_HandleTypeDef VoltageQPR; /**< Voltage-loop resonant controller. */
    NCO_HandleTypeDef NCO;        /**< Phase generator used to create the sinusoidal reference. */
    float NominalFrequency;       /**< Nominal output frequency in Hz. */
    float triggerFrequency;       /**< Control loop frequency in Hz. */
    float InputVoltage;           /**< Fixed inverter input voltage used for feed-forward. */
    float TargetVoltage;          /**< Target output voltage amplitude. */
    float VoltageQPRKb;           /**< QPR back-calculation anti-windup coefficient. */
    OffInv_StateTypeDef State;    /**< Current inverter controller state. */
    OffInv_FaultCodeTypeDef FaultCode; /**< Current protection fault code. */
    float Duty; /**< Latest signed modulation duty before half-cycle leg selection. */
} OffInv_HandleTypeDef;

/**
 * @brief Output over-voltage protection threshold.
 */
#define OFFINV_OVTHRESHOLD 300.0f

/**
 * @brief Output over-current protection threshold.
 */
#define OFFINV_OCTHRESHOLD 30.0f

/**
 * @brief Maximum generated PWM duty.
 */
#define OFFINV_MAX_DUTY 0.90f

/**
 * @brief Sine-reference deadband around zero crossing.
 */
#define OFFINV_ZERO_CROSS_DEADBAND 0.1f

/**
 * @brief Default QPR back-calculation anti-windup coefficient.
 */
#define OFFINV_DEFAULT_QPR_KB 0.5f

/**
 * @brief Initialize an off-grid inverter controller.
 *
 * @param handle Off-grid inverter handle to initialize.
 * @param init Initialization parameters.
 * @return VOLOOP_OK on success, otherwise a VOLOOP error code.
 */
VOLOOP_StatusTypeDef VOLOOP_OffInv_Init(OffInv_HandleTypeDef* handle, OffInv_InitTypeDef* init);

/**
 * @brief Deinitialize an off-grid inverter controller.
 *
 * @param handle Off-grid inverter handle to clear.
 * @return VOLOOP_OK on success, otherwise a VOLOOP error code.
 */
VOLOOP_StatusTypeDef VOLOOP_OffInv_DeInit(OffInv_HandleTypeDef* handle);

/**
 * @brief Start the off-grid inverter controller.
 *
 * This starts the internal NCO, resets the voltage QPR history, and moves the
 * controller to OFFINV_RUNNING.
 *
 * @param handle Off-grid inverter handle.
 * @return VOLOOP_OK on success, otherwise a VOLOOP error code.
 */
VOLOOP_StatusTypeDef VOLOOP_OffInv_Start(OffInv_HandleTypeDef* handle);

/**
 * @brief Stop the off-grid inverter controller.
 *
 * @param handle Off-grid inverter handle.
 * @return VOLOOP_OK on success, otherwise a VOLOOP error code.
 */
VOLOOP_StatusTypeDef VOLOOP_OffInv_Stop(OffInv_HandleTypeDef* handle);

/**
 * @brief Get the current off-grid inverter state.
 *
 * @param handle Off-grid inverter handle.
 * @return Current state, or OFFINV_ERROR if @p handle is NULL.
 */
OffInv_StateTypeDef VOLOOP_OffInv_GetState(OffInv_HandleTypeDef* handle);

/**
 * @brief Get the current off-grid inverter fault code.
 *
 * @param handle Off-grid inverter handle.
 * @return Current fault code, or OFFINV_INVALID if @p handle is NULL.
 */
OffInv_FaultCodeTypeDef VOLOOP_OffInv_GetFaultCode(OffInv_HandleTypeDef* handle);

/**
 * @brief Clear the current off-grid inverter fault code.
 *
 * @param handle Off-grid inverter handle.
 * @return VOLOOP_OK on success, otherwise a VOLOOP error code.
 *
 * @note This function is valid only while the controller is OFFINV_ERROR or
 *       OFFINV_DISABLED. The controller returns to OFFINV_DISABLED on success.
 */
VOLOOP_StatusTypeDef VOLOOP_OffInv_ClearFaultCode(OffInv_HandleTypeDef* handle);

/**
 * @brief Set the target output peak voltage.
 *
 * @param handle Off-grid inverter handle.
 * @param PeakVoltage Positive target output peak voltage.
 * @return VOLOOP_OK on success, otherwise a VOLOOP error code.
 */
VOLOOP_StatusTypeDef VOLOOP_OffInv_SetValue(OffInv_HandleTypeDef* handle, float PeakVoltage);

/**
 * @brief Get the latest signed modulation duty.
 *
 * @param handle Off-grid inverter handle.
 * @return Latest signed modulation duty, or 0.0f if @p handle is NULL.
 */
float VOLOOP_OffInv_GetDuty(OffInv_HandleTypeDef* handle);

/**
 * @brief Run one off-grid inverter control-loop update.
 *
 * This function checks over-current and over-voltage protection, advances the
 * internal NCO phase, computes a sinusoidal voltage reference, runs the voltage
 * QPR controller, and writes bridge-leg PWM commands.
 *
 * @warning This function is intended for high-frequency interrupt/control-loop
 *          use and therefore does not perform full parameter validity checks.
 *          The caller must ensure @p handle, @p input, @p output, and measured
 *          input values are valid before calling.
 *
 * @param handle Off-grid inverter handle.
 * @param input Measured voltage and current input.
 * @param output PWM output command to write.
 * @return VOLOOP_OK on success or disabled-output recommendation,
 *         VOLOOP_ERROR on protection fault, otherwise a VOLOOP error code.
 *
 * @note If the controller is OFFINV_DISABLED, @p output is written with both
 *       legs set to VOLOOP_PWM_DISABLED and zero duty, and the function returns
 *       VOLOOP_OK.
 * @note If the controller is OFFINV_ERROR, @p output is written with both legs
 *       set to VOLOOP_PWM_DISABLED and zero duty, and the function returns
 *       VOLOOP_INVALID_STATE.
 */
VOLOOP_StatusTypeDef VOLOOP_OffInv_Sync(OffInv_HandleTypeDef* handle,
                                        const OffInv_InputTypeDef* input,
                                        OffInv_OutputTypeDef* output);

/** @} */

#endif /* VOLOOP_OFFINV_H */
