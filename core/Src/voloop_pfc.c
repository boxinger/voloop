#include "voloop_pfc.h"

#include <math.h>

#define PFC_GRID_PEAK_EPSILON 1.0e-6f
#define PFC_PEAK_ALPHA_MIN 0.000001f
#define PFC_PEAK_ALPHA_MAX 0.2f
#define PFC_GRID_DC_ALPHA_CYCLES 100.0f
#define PFC_GRID_SQUARE_ALPHA_CYCLES 5.0f

static void VOLOOP_PFC_DisableOutput(PFC_OutputTypeDef* output) {
    output->HighFrequencyPwmState = VOLOOP_PWM_DISABLED;
    output->HighFrequencyDuty = 0.0f;
    output->LinePolarity = PFC_LINE_ZERO;
    output->CurrentReference = 0.0f;
}

static int VOLOOP_PFC_IsFiniteFloat(float value) {
    return isfinite(value);
}

static float VOLOOP_PFC_CalcAlphaByCycles(float triggerFrequency, float nominalFrequency,
                                          float trackCycles) {
    float alpha = VOLOOP_DEF_CalcAlphaByCycles(triggerFrequency, nominalFrequency, trackCycles);

    return VOLOOP_DEF_ClampFloat(alpha, PFC_PEAK_ALPHA_MIN, PFC_PEAK_ALPHA_MAX);
}

static void VOLOOP_PFC_ResetRuntime(PFC_HandleTypeDef* handle) {
    handle->Conductance = 0.0f;
    handle->CurrentReference = 0.0f;
    handle->Duty = 0.0f;
    handle->LinePolarity = PFC_LINE_ZERO;
}

static void VOLOOP_PFC_UpdateGridPeak(PFC_HandleTypeDef* handle, float gridVoltage) {
    float nominalFrequency = handle->PLL.NominalFrequency;
    float dcAlpha = VOLOOP_PFC_CalcAlphaByCycles(handle->triggerFrequency, nominalFrequency,
                                                 PFC_GRID_DC_ALPHA_CYCLES);
    float squareAlpha = VOLOOP_PFC_CalcAlphaByCycles(handle->triggerFrequency, nominalFrequency,
                                                     PFC_GRID_SQUARE_ALPHA_CYCLES);

    handle->GridVoltageDc += dcAlpha * (gridVoltage - handle->GridVoltageDc);
    float acValue = gridVoltage - handle->GridVoltageDc;
    handle->GridVoltageSquareAvg +=
        squareAlpha * ((acValue * acValue) - handle->GridVoltageSquareAvg);
    if (handle->GridVoltageSquareAvg < 0.0f) {
        handle->GridVoltageSquareAvg = 0.0f;
    }
    handle->GridVoltagePeak = sqrtf(handle->GridVoltageSquareAvg * 2.0f);
}

static PFC_LinePolarityTypeDef VOLOOP_PFC_GetPolarityFromSine(float sine) {
    if (sine >= PFC_ZERO_CROSS_DEADBAND) {
        return PFC_LINE_POSITIVE;
    }
    if (sine <= -PFC_ZERO_CROSS_DEADBAND) {
        return PFC_LINE_NEGATIVE;
    }
    return PFC_LINE_ZERO;
}

VOLOOP_StatusTypeDef VOLOOP_PFC_Init(PFC_HandleTypeDef* handle, const PFC_InitTypeDef* init) {
    if (handle == NULL || init == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    if (init->PLLInit == NULL || init->BusVoltagePIDInit == NULL ||
        init->GridCurrentPIDInit == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    if (!VOLOOP_PFC_IsFiniteFloat(init->triggerFrequency) || init->triggerFrequency <= 0.0f) {
        return VOLOOP_INVALID_PARAM;
    }

    *handle = (PFC_HandleTypeDef){ 0 };
    handle->triggerFrequency = init->triggerFrequency;
    handle->TargetBusVoltage = 0.0f;
    handle->MaxGridCurrent = 0.0f;
    handle->GridVoltagePeak = 1.0f;
    handle->GridVoltageDc = 0.0f;
    handle->GridVoltageSquareAvg = 1.0f;
    handle->State = PFC_DISABLED;
    handle->FaultCode = PFC_NOERROR;
    handle->PLL = (PLL_HandleTypeDef){ 0 };
    handle->BusVoltagePID = (PID_HandleTypeDef){ 0 };
    handle->GridCurrentPID = (PID_HandleTypeDef){ 0 };
    VOLOOP_PFC_ResetRuntime(handle);

    VOLOOP_StatusTypeDef status = VOLOOP_PLL_Init(&(handle->PLL), init->PLLInit);
    if (status != VOLOOP_OK) {
        VOLOOP_PFC_DeInit(handle);
        return status;
    }

    status = VOLOOP_PID_Init(&(handle->BusVoltagePID), init->BusVoltagePIDInit);
    if (status != VOLOOP_OK) {
        VOLOOP_PFC_DeInit(handle);
        return status;
    }

    status = VOLOOP_PID_Init(&(handle->GridCurrentPID), init->GridCurrentPIDInit);
    if (status != VOLOOP_OK) {
        VOLOOP_PFC_DeInit(handle);
        return status;
    }

    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_PFC_DeInit(PFC_HandleTypeDef* handle) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    VOLOOP_PLL_DeInit(&(handle->PLL));
    VOLOOP_PID_DeInit(&(handle->BusVoltagePID));
    VOLOOP_PID_DeInit(&(handle->GridCurrentPID));

    *handle = (PFC_HandleTypeDef){ 0 };
    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_PFC_Start(PFC_HandleTypeDef* handle) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    if (handle->State == PFC_WAIT_PLL || handle->State == PFC_RUNNING ||
        handle->State == PFC_CURRENT_LIMIT) {
        return VOLOOP_OK;
    }
    if (handle->State != PFC_DISABLED) {
        return VOLOOP_INVALID_STATE;
    }

    VOLOOP_StatusTypeDef status = VOLOOP_PLL_Start(&(handle->PLL));
    if (status != VOLOOP_OK) {
        handle->State = PFC_ERROR;
        handle->FaultCode = PFC_PLL;
        return status;
    }

    VOLOOP_PID_Reset(&(handle->BusVoltagePID));
    VOLOOP_PID_Reset(&(handle->GridCurrentPID));
    VOLOOP_PFC_ResetRuntime(handle);
    handle->State = PFC_WAIT_PLL;
    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_PFC_Stop(PFC_HandleTypeDef* handle) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    if (handle->State == PFC_DISABLED) {
        return VOLOOP_OK;
    }
    if (handle->State == PFC_ERROR) {
        return VOLOOP_INVALID_STATE;
    }

    VOLOOP_StatusTypeDef status = VOLOOP_PLL_Stop(&(handle->PLL));
    if (status != VOLOOP_OK) {
        handle->State = PFC_ERROR;
        handle->FaultCode = PFC_PLL;
        return status;
    }

    handle->State = PFC_DISABLED;
    VOLOOP_PFC_ResetRuntime(handle);
    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_PFC_ClearFaultCode(PFC_HandleTypeDef* handle) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    if (handle->State != PFC_ERROR && handle->State != PFC_DISABLED) {
        return VOLOOP_INVALID_STATE;
    }

    handle->FaultCode = PFC_NOERROR;
    handle->State = PFC_DISABLED;
    VOLOOP_PFC_ResetRuntime(handle);
    return VOLOOP_OK;
}

PFC_StateTypeDef VOLOOP_PFC_GetState(const PFC_HandleTypeDef* handle) {
    if (handle == NULL) {
        return PFC_ERROR;
    }
    return handle->State;
}

PFC_FaultCodeTypeDef VOLOOP_PFC_GetFaultCode(const PFC_HandleTypeDef* handle) {
    if (handle == NULL) {
        return PFC_INVALID;
    }
    return handle->FaultCode;
}

PLL_LockStateTypeDef VOLOOP_PFC_GetPLLLockState(const PFC_HandleTypeDef* handle) {
    if (handle == NULL) {
        return PLL_UNLOCKED;
    }
    return VOLOOP_PLL_IsLocked(&(handle->PLL));
}

VOLOOP_StatusTypeDef VOLOOP_PFC_SetValue(PFC_HandleTypeDef* handle, float busVoltage,
                                         float maxGridCurrent) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    if (!VOLOOP_PFC_IsFiniteFloat(busVoltage) || !VOLOOP_PFC_IsFiniteFloat(maxGridCurrent) ||
        busVoltage <= 0.0f || busVoltage > PFC_BUS_OVTHRESHOLD || maxGridCurrent <= 0.0f ||
        maxGridCurrent > PFC_INPUT_OCTHRESHOLD) {
        return VOLOOP_INVALID_PARAM;
    }

    handle->TargetBusVoltage = busVoltage;
    handle->MaxGridCurrent = maxGridCurrent;
    return VOLOOP_OK;
}

float VOLOOP_PFC_GetDuty(const PFC_HandleTypeDef* handle) {
    if (handle == NULL) {
        return 0.0f;
    }
    return handle->Duty;
}

float VOLOOP_PFC_GetCurrentReference(const PFC_HandleTypeDef* handle) {
    if (handle == NULL) {
        return 0.0f;
    }
    return handle->CurrentReference;
}

float VOLOOP_PFC_GetGridVoltagePeak(const PFC_HandleTypeDef* handle) {
    if (handle == NULL) {
        return 0.0f;
    }
    return handle->GridVoltagePeak;
}

PFC_LinePolarityTypeDef VOLOOP_PFC_GetLinePolarity(const PFC_HandleTypeDef* handle) {
    if (handle == NULL) {
        return PFC_LINE_ZERO;
    }
    return handle->LinePolarity;
}

VOLOOP_StatusTypeDef VOLOOP_PFC_Sync(PFC_HandleTypeDef* handle, const PFC_InputTypeDef* input,
                                     PFC_OutputTypeDef* output) {
    if (handle == NULL || input == NULL || output == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    if (handle->State == PFC_ERROR) {
        VOLOOP_PFC_DisableOutput(output);
        return VOLOOP_INVALID_STATE;
    }
    if (handle->State == PFC_DISABLED) {
        VOLOOP_PFC_DisableOutput(output);
        return VOLOOP_OK;
    }

    if (fabsf(input->GridCurrent) > PFC_INPUT_OCTHRESHOLD) {
        handle->State = PFC_ERROR;
        handle->FaultCode = PFC_OCP;
        VOLOOP_PFC_DisableOutput(output);
        return VOLOOP_ERROR;
    }
    if (input->BusVoltage > PFC_BUS_OVTHRESHOLD) {
        handle->State = PFC_ERROR;
        handle->FaultCode = PFC_OVP;
        VOLOOP_PFC_DisableOutput(output);
        return VOLOOP_ERROR;
    }

    PLL_InputTypeDef pllInput = {
        .InputVoltage = input->GridVoltage,
    };
    VOLOOP_StatusTypeDef status = VOLOOP_PLL_Sync(&(handle->PLL), &pllInput);
    if (status != VOLOOP_OK || VOLOOP_PLL_GetState(&(handle->PLL)) == PLL_ERROR) {
        handle->State = PFC_ERROR;
        handle->FaultCode = PFC_PLL;
        VOLOOP_PFC_DisableOutput(output);
        return (status == VOLOOP_OK) ? VOLOOP_ERROR : status;
    }

    VOLOOP_PFC_UpdateGridPeak(handle, input->GridVoltage);

    if (VOLOOP_PLL_IsLocked(&(handle->PLL)) != PLL_LOCKED ||
        handle->GridVoltagePeak <= PFC_GRID_PEAK_EPSILON) {
        handle->State = PFC_WAIT_PLL;
        handle->CurrentReference = 0.0f;
        handle->LinePolarity = PFC_LINE_ZERO;
        VOLOOP_PFC_DisableOutput(output);
        return VOLOOP_OK;
    }

    int32_t phaseQ31 = VOLOOP_PLL_GetPhaseQ31(&(handle->PLL));
    float sine = VOLOOP_DEF_SIN(phaseQ31);
    PFC_LinePolarityTypeDef linePolarity = VOLOOP_PFC_GetPolarityFromSine(sine);

    float maxConductance = handle->MaxGridCurrent / handle->GridVoltagePeak;
    float conductance =
        VOLOOP_PID_ComputeConditional(&(handle->BusVoltagePID), handle->TargetBusVoltage,
                                      input->BusVoltage, 0.0f, maxConductance);
    handle->Conductance = conductance;
    handle->State = (VOLOOP_PID_GetState(&(handle->BusVoltagePID)) == PID_UpperSaturated)
                        ? PFC_CURRENT_LIMIT
                        : PFC_RUNNING;

    float currentReference = conductance * handle->GridVoltagePeak * sine;
    currentReference =
        VOLOOP_DEF_ClampFloat(currentReference, -handle->MaxGridCurrent, handle->MaxGridCurrent);
    handle->CurrentReference = currentReference;
    handle->LinePolarity = linePolarity;

    if (linePolarity == PFC_LINE_ZERO) {
        output->HighFrequencyPwmState = VOLOOP_PWM_DISABLED;
        output->HighFrequencyDuty = 0.0f;
        output->LinePolarity = PFC_LINE_ZERO;
        output->CurrentReference = currentReference;
        return VOLOOP_OK;
    }

    float currentReferenceMagnitude = fabsf(currentReference);
    float currentMeasurementAligned =
        (linePolarity == PFC_LINE_POSITIVE) ? input->GridCurrent : -input->GridCurrent;
    float duty =
        VOLOOP_PID_ComputeConditional(&(handle->GridCurrentPID), currentReferenceMagnitude,
                                      currentMeasurementAligned, PFC_MIN_DUTY, PFC_MAX_DUTY);

    handle->Duty = duty;
    output->HighFrequencyPwmState = VOLOOP_PWM_ENABLE;
    output->HighFrequencyDuty = duty;
    output->LinePolarity = linePolarity;
    output->CurrentReference = currentReference;

    return VOLOOP_OK;
}
