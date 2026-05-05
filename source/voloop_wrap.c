#include "voloop_wrap.h"

/* ===== PID Raw-ADC Wrappers ===== */

float VOLOOP_Wrap_PID_ComputeADC(PID_HandleTypeDef* pid,
                                 uint32_t adcSetpoint, uint32_t adcMeasurement,
                                 const VOLOOP_ScaleTypeDef* scale)
{
    float sp = VOLOOP_Scale_RawToPhys(adcSetpoint, scale);
    float mv = VOLOOP_Scale_RawToPhys(adcMeasurement, scale);
    return VOLOOP_PID_Compute(pid, sp, mv);
}

float VOLOOP_Wrap_PID_ComputeConditionalADC(PID_HandleTypeDef* pid,
                                            uint32_t adcSetpoint, uint32_t adcMeasurement,
                                            float outputMin, float outputMax,
                                            const VOLOOP_ScaleTypeDef* scale)
{
    float sp = VOLOOP_Scale_RawToPhys(adcSetpoint, scale);
    float mv = VOLOOP_Scale_RawToPhys(adcMeasurement, scale);
    return VOLOOP_PID_ComputeConditional(pid, sp, mv, outputMin, outputMax);
}

float VOLOOP_Wrap_PID_ComputeBackCalcADC(PID_HandleTypeDef* pid,
                                         uint32_t adcSetpoint, uint32_t adcMeasurement,
                                         float outputMin, float outputMax,
                                         float antiWindupGain,
                                         const VOLOOP_ScaleTypeDef* scale)
{
    float sp = VOLOOP_Scale_RawToPhys(adcSetpoint, scale);
    float mv = VOLOOP_Scale_RawToPhys(adcMeasurement, scale);
    return VOLOOP_PID_ComputeBackCalculation(pid, sp, mv, outputMin, outputMax, antiWindupGain);
}

/* ===== PID Typed Wrappers ===== */

float VOLOOP_Wrap_PID_Compute_V(PID_HandleTypeDef* pid,
                                VOLOOP_Voltage setpoint,
                                VOLOOP_Voltage measurement)
{
    return VOLOOP_PID_Compute(pid, setpoint.value, measurement.value);
}

float VOLOOP_Wrap_PID_Compute_C(PID_HandleTypeDef* pid,
                                VOLOOP_Current setpoint,
                                VOLOOP_Current measurement)
{
    return VOLOOP_PID_Compute(pid, setpoint.value, measurement.value);
}

float VOLOOP_Wrap_PID_Compute_D(PID_HandleTypeDef* pid,
                                VOLOOP_Duty setpoint,
                                VOLOOP_Duty measurement)
{
    return VOLOOP_PID_Compute(pid, setpoint.value, measurement.value);
}

float VOLOOP_Wrap_PID_ComputeConditional_V(PID_HandleTypeDef* pid,
                                           VOLOOP_Voltage setpoint,
                                           VOLOOP_Voltage measurement,
                                           float outputMin, float outputMax)
{
    return VOLOOP_PID_ComputeConditional(pid, setpoint.value, measurement.value, outputMin, outputMax);
}

float VOLOOP_Wrap_PID_ComputeConditional_C(PID_HandleTypeDef* pid,
                                           VOLOOP_Current setpoint,
                                           VOLOOP_Current measurement,
                                           float outputMin, float outputMax)
{
    return VOLOOP_PID_ComputeConditional(pid, setpoint.value, measurement.value, outputMin, outputMax);
}

float VOLOOP_Wrap_PID_ComputeConditional_D(PID_HandleTypeDef* pid,
                                           VOLOOP_Duty setpoint,
                                           VOLOOP_Duty measurement,
                                           float outputMin, float outputMax)
{
    return VOLOOP_PID_ComputeConditional(pid, setpoint.value, measurement.value, outputMin, outputMax);
}

float VOLOOP_Wrap_PID_ComputeBackCalc_V(PID_HandleTypeDef* pid,
                                        VOLOOP_Voltage setpoint,
                                        VOLOOP_Voltage measurement,
                                        float outputMin, float outputMax,
                                        float antiWindupGain)
{
    return VOLOOP_PID_ComputeBackCalculation(pid, setpoint.value, measurement.value, outputMin, outputMax, antiWindupGain);
}

float VOLOOP_Wrap_PID_ComputeBackCalc_C(PID_HandleTypeDef* pid,
                                        VOLOOP_Current setpoint,
                                        VOLOOP_Current measurement,
                                        float outputMin, float outputMax,
                                        float antiWindupGain)
{
    return VOLOOP_PID_ComputeBackCalculation(pid, setpoint.value, measurement.value, outputMin, outputMax, antiWindupGain);
}

float VOLOOP_Wrap_PID_ComputeBackCalc_D(PID_HandleTypeDef* pid,
                                        VOLOOP_Duty setpoint,
                                        VOLOOP_Duty measurement,
                                        float outputMin, float outputMax,
                                        float antiWindupGain)
{
    return VOLOOP_PID_ComputeBackCalculation(pid, setpoint.value, measurement.value, outputMin, outputMax, antiWindupGain);
}

/* ===== Buck Scaled Wrapper ===== */

static VOLOOP_WrapBuckTypeDef* g_wbActive;

static float _wbGetVoltage(void)
{
    return VOLOOP_Scale_RawToPhys(g_wbActive->rawGetVoltage(), &g_wbActive->vScale);
}

static float _wbGetCurrent(void)
{
    return VOLOOP_Scale_RawToPhys(g_wbActive->rawGetCurrent(), &g_wbActive->cScale);
}

VOLOOP_StatusTypeDef VOLOOP_WrapBuck_Init(VOLOOP_WrapBuckTypeDef* wb,
                                          Buck_InitTypeDef* init,
                                          const VOLOOP_ScaleTypeDef* vScale,
                                          const VOLOOP_ScaleTypeDef* cScale,
                                          uint32_t (*rawGetVoltage)(void),
                                          uint32_t (*rawGetCurrent)(void))
{
    if (wb == NULL || init == NULL || vScale == NULL || cScale == NULL
        || rawGetVoltage == NULL || rawGetCurrent == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    wb->init = *init;
    wb->init.GetOutputVoltage = _wbGetVoltage;
    wb->init.GetInductorCurrent = _wbGetCurrent;
    wb->vScale = *vScale;
    wb->cScale = *cScale;
    wb->rawGetVoltage = rawGetVoltage;
    wb->rawGetCurrent = rawGetCurrent;

    return VOLOOP_Buck_InitStatic(&wb->buck, &wb->init);
}

VOLOOP_StatusTypeDef VOLOOP_WrapBuck_DeInit(VOLOOP_WrapBuckTypeDef* wb)
{
    if (wb == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    return VOLOOP_Buck_DeInitStatic(&wb->buck);
}

VOLOOP_StatusTypeDef VOLOOP_WrapBuck_Sync(VOLOOP_WrapBuckTypeDef* wb)
{
    if (wb == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    g_wbActive = wb;
    VOLOOP_StatusTypeDef status = VOLOOP_Buck_Sync(&wb->buck);
    g_wbActive = NULL;
    return status;
}

/* ===== Signal Scaled Wrappers ===== */

float VOLOOP_Wrap_SlewLimiter_ADC(VOLOOP_SlewLimiterTypeDef* slew,
                                  uint32_t adcTarget, float dt,
                                  const VOLOOP_ScaleTypeDef* scale)
{
    float target = VOLOOP_Scale_RawToPhys(adcTarget, scale);
    return VOLOOP_Signal_SlewLimiter_Update(slew, target, dt);
}

float VOLOOP_Wrap_MAF_UpdateADC(VOLOOP_MAFTypeDef* maf,
                                uint32_t adcInput,
                                const VOLOOP_ScaleTypeDef* scale)
{
    float input = VOLOOP_Scale_RawToPhys(adcInput, scale);
    return VOLOOP_Signal_MAF_Update(maf, input);
}

float VOLOOP_Wrap_EMAF_UpdateADC(VOLOOP_EMAFTypeDef* ema,
                                 uint32_t adcInput,
                                 const VOLOOP_ScaleTypeDef* scale)
{
    float input = VOLOOP_Scale_RawToPhys(adcInput, scale);
    return VOLOOP_Signal_EMAF_Update(ema, input);
}
