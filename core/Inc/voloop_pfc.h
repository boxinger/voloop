/**
 * @file voloop_pfc.h
 * @brief Single-phase totem-pole PFC control API.
 */
#ifndef VOLOOP_PFC_H
#define VOLOOP_PFC_H

#include "voloop_def.h"
#include "voloop_pid.h"
#include "voloop_pll.h"

/**
 * @defgroup VOLOOP_PFC Totem-Pole PFC
 * @ingroup VOLOOP_CORE
 * @brief Single-phase totem-pole PFC control APIs built from PLL and cascaded PID loops.
 *
 * The PFC module combines an internal PLL, an outer DC-bus voltage loop, and
 * an inner grid-current loop. It estimates the grid phase and peak voltage,
 * generates a sinusoidal signed current reference, and outputs a high-frequency
 * PWM duty recommendation plus line-polarity information for board code.
 *
 * The module does not read ADC values or drive PWM hardware directly.
 *
 * ## Basic usage
 *
 * 1. Prepare ::PLL_InitTypeDef and two ::PID_InitTypeDef objects.
 * 2. Fill a ::PFC_InitTypeDef object.
 * 3. Call ::VOLOOP_PFC_Init and ::VOLOOP_PFC_Start.
 * 4. Set the DC-bus target and grid-current limit with ::VOLOOP_PFC_SetValue.
 * 5. Call ::VOLOOP_PFC_Sync once per control-loop sample.
 * 6. Apply the returned ::PFC_OutputTypeDef PWM recommendation in board code.
 *
 * @{
 */

/**
 * @brief PFC initialization parameters.
 */
typedef struct {
    const PLL_InitTypeDef* PLLInit;            /**< PLL initialization for grid synchronization. */
    const PID_InitTypeDef* BusVoltagePIDInit;  /**< Outer DC-bus voltage-loop PID initialization. */
    const PID_InitTypeDef* GridCurrentPIDInit; /**< Inner grid-current-loop PID initialization. */
    float triggerFrequency;                    /**< Control loop frequency in Hz. */
} PFC_InitTypeDef;

/**
 * @brief PFC measured input sample.
 */
typedef struct {
    float GridVoltage; /**< Signed AC input voltage sample. */
    float GridCurrent; /**< Signed AC/input inductor current sample. */
    float BusVoltage;  /**< DC bus voltage sample. */
} PFC_InputTypeDef;

/**
 * @brief Line-polarity recommendation for the totem-pole slow leg.
 */
typedef enum {
    PFC_LINE_ZERO = 0U, /**< Grid phase is inside the zero-crossing deadband. */
    PFC_LINE_POSITIVE,  /**< Positive grid half-cycle. */
    PFC_LINE_NEGATIVE   /**< Negative grid half-cycle. */
} PFC_LinePolarityTypeDef;

/**
 * @brief PFC PWM output recommendation.
 */
typedef struct {
    VOLOOP_DEF_PwmStateTypeDef HighFrequencyPwmState; /**< High-frequency PWM state. */
    float HighFrequencyDuty;                          /**< High-frequency duty recommendation. */
    PFC_LinePolarityTypeDef LinePolarity;             /**< Slow-leg polarity recommendation. */
    float CurrentReference;                           /**< Latest signed grid-current reference. */
} PFC_OutputTypeDef;

/**
 * @brief PFC runtime state.
 */
typedef enum {
    PFC_ERROR = 0U,   /**< Error state, usually caused by protection or child-module failure. */
    PFC_DISABLED,     /**< PFC controller is initialized but not running. */
    PFC_WAIT_PLL,     /**< PFC controller is running but PLL is not locked. */
    PFC_RUNNING,      /**< PFC controller is running without current-limit saturation. */
    PFC_CURRENT_LIMIT /**< Voltage-loop conductance is clamped by the configured current limit. */
} PFC_StateTypeDef;

/**
 * @brief PFC protection fault code.
 */
typedef enum {
    PFC_INVALID = 0U, /**< Invalid fault code, also returned for invalid handles. */
    PFC_NOERROR,      /**< No active fault. */
    PFC_OCP,          /**< Grid/input over-current protection fault. */
    PFC_OVP,          /**< DC-bus over-voltage protection fault. */
    PFC_PLL           /**< PLL child-module fault. */
} PFC_FaultCodeTypeDef;

/**
 * @brief PFC runtime handle.
 *
 * The handle owns the PLL, voltage-loop PID, current-loop PID, latest targets,
 * grid-voltage peak estimator, generated current reference, fault state, and
 * latest duty recommendation.
 */
typedef struct {
    PLL_HandleTypeDef PLL;                /**< Internal grid PLL. */
    PID_HandleTypeDef BusVoltagePID;      /**< Outer DC-bus voltage-loop PID controller. */
    PID_HandleTypeDef GridCurrentPID;     /**< Inner grid-current-loop PID controller. */
    float triggerFrequency;               /**< Control loop frequency in Hz. */
    float TargetBusVoltage;               /**< Target DC bus voltage. */
    float MaxGridCurrent;                 /**< Maximum absolute grid-current reference. */
    float GridVoltagePeak;                /**< Estimated grid voltage peak. */
    float GridVoltageDc;                  /**< Estimated grid voltage DC offset. */
    float GridVoltageSquareAvg;           /**< Estimated grid voltage AC mean square. */
    float Conductance;                    /**< Latest limited input conductance command. */
    float CurrentReference;               /**< Latest signed grid-current reference. */
    float Duty;                           /**< Latest active high-frequency duty recommendation. */
    PFC_LinePolarityTypeDef LinePolarity; /**< Latest line-polarity recommendation. */
    PFC_StateTypeDef State;               /**< Current PFC controller state. */
    PFC_FaultCodeTypeDef FaultCode;       /**< Current protection fault code. */
} PFC_HandleTypeDef;

/**
 * @brief DC bus over-voltage protection threshold.
 */
#define PFC_BUS_OVTHRESHOLD 50.0f

/**
 * @brief Input/grid over-current protection threshold.
 */
#define PFC_INPUT_OCTHRESHOLD 5.0f

/**
 * @brief Maximum generated PWM duty.
 */
#define PFC_MAX_DUTY 0.95f

/**
 * @brief Minimum generated PWM duty.
 */
#define PFC_MIN_DUTY 0.0f

/**
 * @brief Sine-reference deadband around zero crossing.
 */
#define PFC_ZERO_CROSS_DEADBAND 0.05f

/**
 * @brief Initialize a PFC controller.
 *
 * @param handle PFC handle to initialize.
 * @param init Initialization parameters.
 * @return VOLOOP_OK on success, otherwise a VOLOOP error code.
 */
VOLOOP_StatusTypeDef VOLOOP_PFC_Init(PFC_HandleTypeDef* handle, const PFC_InitTypeDef* init);

/**
 * @brief Deinitialize a PFC controller.
 *
 * @param handle PFC handle to clear.
 * @return VOLOOP_OK on success, otherwise a VOLOOP error code.
 */
VOLOOP_StatusTypeDef VOLOOP_PFC_DeInit(PFC_HandleTypeDef* handle);

/**
 * @brief Start the PFC controller.
 *
 * This starts the internal PLL, resets both PID controllers, clears runtime
 * commands, and moves the controller to PFC_WAIT_PLL.
 *
 * @param handle PFC handle.
 * @return VOLOOP_OK on success, otherwise a VOLOOP error code.
 */
VOLOOP_StatusTypeDef VOLOOP_PFC_Start(PFC_HandleTypeDef* handle);

/**
 * @brief Stop the PFC controller.
 *
 * @param handle PFC handle.
 * @return VOLOOP_OK on success, otherwise a VOLOOP error code.
 */
VOLOOP_StatusTypeDef VOLOOP_PFC_Stop(PFC_HandleTypeDef* handle);

/**
 * @brief Clear the current PFC fault code.
 *
 * @param handle PFC handle.
 * @return VOLOOP_OK on success, otherwise a VOLOOP error code.
 *
 * @note This function is valid only while the controller is PFC_ERROR or
 *       PFC_DISABLED. The controller returns to PFC_DISABLED on success.
 */
VOLOOP_StatusTypeDef VOLOOP_PFC_ClearFaultCode(PFC_HandleTypeDef* handle);

/**
 * @brief Get the current PFC runtime state.
 *
 * @param handle PFC handle.
 * @return Current state, or PFC_ERROR if @p handle is NULL.
 */
PFC_StateTypeDef VOLOOP_PFC_GetState(const PFC_HandleTypeDef* handle);

/**
 * @brief Get the current PFC fault code.
 *
 * @param handle PFC handle.
 * @return Current fault code, or PFC_INVALID if @p handle is NULL.
 */
PFC_FaultCodeTypeDef VOLOOP_PFC_GetFaultCode(const PFC_HandleTypeDef* handle);

/**
 * @brief Get the internal PLL lock state.
 *
 * @param handle PFC handle.
 * @return PLL lock state, or PLL_UNLOCKED if @p handle is NULL.
 */
PLL_LockStateTypeDef VOLOOP_PFC_GetPLLLockState(const PFC_HandleTypeDef* handle);

/**
 * @brief Set the target DC bus voltage and grid-current limit.
 *
 * @param handle PFC handle.
 * @param busVoltage Positive target DC bus voltage.
 * @param maxGridCurrent Positive maximum absolute grid-current reference.
 * @return VOLOOP_OK on success, otherwise a VOLOOP error code.
 */
VOLOOP_StatusTypeDef VOLOOP_PFC_SetValue(PFC_HandleTypeDef* handle, float busVoltage,
                                         float maxGridCurrent);

/**
 * @brief Get the latest active high-frequency duty recommendation.
 *
 * @param handle PFC handle.
 * @return Latest active duty recommendation, or 0.0f if @p handle is NULL.
 */
float VOLOOP_PFC_GetDuty(const PFC_HandleTypeDef* handle);

/**
 * @brief Get the latest signed grid-current reference.
 *
 * @param handle PFC handle.
 * @return Latest current reference, or 0.0f if @p handle is NULL.
 */
float VOLOOP_PFC_GetCurrentReference(const PFC_HandleTypeDef* handle);

/**
 * @brief Get the latest estimated grid-voltage peak.
 *
 * @param handle PFC handle.
 * @return Latest grid-voltage peak estimate, or 0.0f if @p handle is NULL.
 */
float VOLOOP_PFC_GetGridVoltagePeak(const PFC_HandleTypeDef* handle);

/**
 * @brief Get the latest line-polarity recommendation.
 *
 * @param handle PFC handle.
 * @return Latest line polarity, or PFC_LINE_ZERO if @p handle is NULL.
 */
PFC_LinePolarityTypeDef VOLOOP_PFC_GetLinePolarity(const PFC_HandleTypeDef* handle);

/**
 * @brief Run one PFC control-loop update.
 *
 * This function checks protection, updates the internal PLL, waits for lock,
 * estimates grid voltage peak, computes a conductance command from the bus
 * voltage loop, generates a sinusoidal current reference, freezes the current
 * loop inside the zero-crossing deadband, and writes a PWM recommendation.
 *
 * @param handle PFC handle.
 * @param input Measured voltage/current input.
 * @param output PWM output recommendation to write.
 * @return VOLOOP_OK on success or disabled-output recommendation,
 *         VOLOOP_ERROR on protection fault, otherwise a VOLOOP error code.
 *
 * @note If the controller is PFC_DISABLED, @p output is written with disabled
 *       PWM and zero duty, and the function returns VOLOOP_OK.
 * @note If the controller is PFC_ERROR, @p output is written with disabled PWM
 *       and zero duty, and the function returns VOLOOP_INVALID_STATE.
 */
VOLOOP_StatusTypeDef VOLOOP_PFC_Sync(PFC_HandleTypeDef* handle, const PFC_InputTypeDef* input,
                                     PFC_OutputTypeDef* output);

/** @} */

#endif /* VOLOOP_PFC_H */
