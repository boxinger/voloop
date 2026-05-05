#ifndef VOLOOP_WRAP_H
#define VOLOOP_WRAP_H

#include "voloop_pid.h"
#include "voloop_buck.h"
#include "voloop_scale.h"
#include "voloop_signal.h"

/* ====================================================================
   Wrap functions: thin adapters that connect Scale ↔ Algorithm cores.
   Core algorithms are NOT modified — these are pure composition layers.
   ==================================================================== */

/* ===== PID Raw-ADC Wrappers ===== */
/* Accept raw ADC counts + scale config → internally convert to
   physical values → call real PID compute → return float output. */

float VOLOOP_Wrap_PID_ComputeADC(PID_HandleTypeDef* pid,
                                 uint32_t adcSetpoint, uint32_t adcMeasurement,
                                 const VOLOOP_ScaleTypeDef* scale);

float VOLOOP_Wrap_PID_ComputeConditionalADC(PID_HandleTypeDef* pid,
                                            uint32_t adcSetpoint, uint32_t adcMeasurement,
                                            float outputMin, float outputMax,
                                            const VOLOOP_ScaleTypeDef* scale);

float VOLOOP_Wrap_PID_ComputeBackCalcADC(PID_HandleTypeDef* pid,
                                         uint32_t adcSetpoint, uint32_t adcMeasurement,
                                         float outputMin, float outputMax,
                                         float antiWindupGain,
                                         const VOLOOP_ScaleTypeDef* scale);

/* ===== PID Typed Wrappers ===== */
/* Compile-time type safety: if you pass VOLOOP_Current where
   VOLOOP_Voltage is expected, the compiler rejects it.
   All return float (algorithm output is dimensionless). */

float VOLOOP_Wrap_PID_Compute_V(PID_HandleTypeDef* pid,
                                VOLOOP_Voltage setpoint,
                                VOLOOP_Voltage measurement);

float VOLOOP_Wrap_PID_Compute_C(PID_HandleTypeDef* pid,
                                VOLOOP_Current setpoint,
                                VOLOOP_Current measurement);

float VOLOOP_Wrap_PID_Compute_D(PID_HandleTypeDef* pid,
                                VOLOOP_Duty setpoint,
                                VOLOOP_Duty measurement);

float VOLOOP_Wrap_PID_ComputeConditional_V(PID_HandleTypeDef* pid,
                                           VOLOOP_Voltage setpoint,
                                           VOLOOP_Voltage measurement,
                                           float outputMin, float outputMax);

float VOLOOP_Wrap_PID_ComputeConditional_C(PID_HandleTypeDef* pid,
                                           VOLOOP_Current setpoint,
                                           VOLOOP_Current measurement,
                                           float outputMin, float outputMax);

float VOLOOP_Wrap_PID_ComputeConditional_D(PID_HandleTypeDef* pid,
                                           VOLOOP_Duty setpoint,
                                           VOLOOP_Duty measurement,
                                           float outputMin, float outputMax);

float VOLOOP_Wrap_PID_ComputeBackCalc_V(PID_HandleTypeDef* pid,
                                        VOLOOP_Voltage setpoint,
                                        VOLOOP_Voltage measurement,
                                        float outputMin, float outputMax,
                                        float antiWindupGain);

float VOLOOP_Wrap_PID_ComputeBackCalc_C(PID_HandleTypeDef* pid,
                                        VOLOOP_Current setpoint,
                                        VOLOOP_Current measurement,
                                        float outputMin, float outputMax,
                                        float antiWindupGain);

float VOLOOP_Wrap_PID_ComputeBackCalc_D(PID_HandleTypeDef* pid,
                                        VOLOOP_Duty setpoint,
                                        VOLOOP_Duty measurement,
                                        float outputMin, float outputMax,
                                        float antiWindupGain);

/* ===== Buck Scaled Wrapper ===== */
/* Wraps a Buck_HandleTypeDef with scale config so that Buck_Sync
   can be driven from raw ADC reads, with automatic physical conversion.
   NOTE: single-instance only — the wrapper uses a file-static pointer. */

typedef struct {
    Buck_HandleTypeDef    buck;
    Buck_InitTypeDef      init;
    VOLOOP_ScaleTypeDef   vScale;
    VOLOOP_ScaleTypeDef   cScale;
    uint32_t            (*rawGetVoltage)(void);
    uint32_t            (*rawGetCurrent)(void);
} VOLOOP_WrapBuckTypeDef;

VOLOOP_StatusTypeDef VOLOOP_WrapBuck_Init(VOLOOP_WrapBuckTypeDef* wb,
                                          Buck_InitTypeDef* init,
                                          const VOLOOP_ScaleTypeDef* vScale,
                                          const VOLOOP_ScaleTypeDef* cScale,
                                          uint32_t (*rawGetVoltage)(void),
                                          uint32_t (*rawGetCurrent)(void));

VOLOOP_StatusTypeDef VOLOOP_WrapBuck_DeInit(VOLOOP_WrapBuckTypeDef* wb);
VOLOOP_StatusTypeDef VOLOOP_WrapBuck_Sync(VOLOOP_WrapBuckTypeDef* wb);

/* ===== Signal Scaled Wrappers ===== */
/* Signal processing with integrated ADC scaling. */

float VOLOOP_Wrap_SlewLimiter_ADC(VOLOOP_SlewLimiterTypeDef* slew,
                                  uint32_t adcTarget, float dt,
                                  const VOLOOP_ScaleTypeDef* scale);

float VOLOOP_Wrap_MAF_UpdateADC(VOLOOP_MAFTypeDef* maf,
                                uint32_t adcInput,
                                const VOLOOP_ScaleTypeDef* scale);

float VOLOOP_Wrap_EMAF_UpdateADC(VOLOOP_EMAFTypeDef* ema,
                                 uint32_t adcInput,
                                 const VOLOOP_ScaleTypeDef* scale);

#endif /* VOLOOP_WRAP_H */
