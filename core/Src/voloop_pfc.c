#include "voloop_pfc.h"

#include <math.h>

static int VOLOOP_PFC_IsFiniteFloat(float value) {
    return isfinite(value);
}

static int VOLOOP_PFC_IsConfigValid(const PFC_ConfigTypeDef* config) {
    if (config == NULL) {
        return 0;
    }

    if (!VOLOOP_PFC_IsFiniteFloat(config->BusOverVoltageThreshold) ||
        config->BusOverVoltageThreshold <= 0.0f) {
        return 0;
    }
    if (!VOLOOP_PFC_IsFiniteFloat(config->GridOverCurrentThreshold) ||
        config->GridOverCurrentThreshold <= 0.0f) {
        return 0;
    }
    if (!VOLOOP_PFC_IsFiniteFloat(config->MinHighFrequencyDuty) ||
        !VOLOOP_PFC_IsFiniteFloat(config->MaxHighFrequencyDuty) ||
        config->MinHighFrequencyDuty < 0.0f ||
        config->MinHighFrequencyDuty > config->MaxHighFrequencyDuty ||
        config->MaxHighFrequencyDuty >= 1.0f) {
        return 0;
    }
    if (!VOLOOP_PFC_IsFiniteFloat(config->ZeroCrossingDeadband) ||
        config->ZeroCrossingDeadband < 0.0f || config->ZeroCrossingDeadband >= 1.0f) {
        return 0;
    }

    return 1;
}

static void VOLOOP_PFC_ResetRuntime(PFC_HandleTypeDef* handle) {
    handle->CurrentAmplitudeReference = 0.0f;
    handle->CurrentReference = 0.0f;
    handle->Modulation = 0.0f;
    handle->LinePolarity = PFC_LINE_ZERO;
}

static VOLOOP_StatusTypeDef VOLOOP_PFC_ResetControllers(PFC_HandleTypeDef* handle) {
    VOLOOP_StatusTypeDef status = VOLOOP_PID_Reset(&(handle->BusVoltagePID));
    if (status != VOLOOP_OK) {
        return status;
    }

    return VOLOOP_PID_Reset(&(handle->GridCurrentPID));
}

static void VOLOOP_PFC_EnterFault(PFC_HandleTypeDef* handle, PFC_FaultCodeTypeDef faultCode) {
    if (VOLOOP_PLL_GetState(&(handle->PLL)) == PLL_RUNNING) {
        (void)VOLOOP_PLL_Stop(&(handle->PLL));
    }

    (void)VOLOOP_PFC_ResetControllers(handle);
    VOLOOP_PFC_ResetRuntime(handle);
    handle->FaultCode = faultCode;
    handle->State = PFC_ERROR;
}

static void VOLOOP_PFC_DisableOutput(PFC_OutputTypeDef* output) {
    output->LeftLegPwmState = VOLOOP_PWM_DISABLED;
    output->LeftLegDuty = 0.0f;
    output->RightLegPwmState = VOLOOP_PWM_DISABLED;
    output->RightLegDuty = 0.0f;
}

static PFC_LinePolarityTypeDef VOLOOP_PFC_GetLinePolarityFromSine(float sine,
                                                                  float zeroCrossingDeadband) {
    if (sine > zeroCrossingDeadband) {
        return PFC_LINE_POSITIVE;
    }
    if (sine < -zeroCrossingDeadband) {
        return PFC_LINE_NEGATIVE;
    }
    return PFC_LINE_ZERO;
}

static VOLOOP_StatusTypeDef VOLOOP_PFC_EnsurePLLRunning(PFC_HandleTypeDef* handle) {
    PLL_StateTypeDef pllState = VOLOOP_PLL_GetState(&(handle->PLL));

    if (pllState == PLL_RUNNING) {
        return VOLOOP_OK;
    }
    if (pllState == PLL_ERROR) {
        VOLOOP_PFC_EnterFault(handle, PFC_FAULT_PLL);
        return VOLOOP_ERROR;
    }
    if (pllState == PLL_RESET) {
        return VOLOOP_INVALID_STATE;
    }

    VOLOOP_StatusTypeDef status = VOLOOP_PLL_Start(&(handle->PLL));
    if (status != VOLOOP_OK && VOLOOP_PLL_GetState(&(handle->PLL)) == PLL_ERROR) {
        VOLOOP_PFC_EnterFault(handle, PFC_FAULT_PLL);
    }
    return status;
}

static void VOLOOP_PFC_ClearHandle(PFC_HandleTypeDef* handle) {
    (void)VOLOOP_PLL_DeInit(&(handle->PLL));
    (void)VOLOOP_PID_DeInit(&(handle->BusVoltagePID));
    (void)VOLOOP_PID_DeInit(&(handle->GridCurrentPID));
    *handle = (PFC_HandleTypeDef){ 0 };
}

VOLOOP_StatusTypeDef VOLOOP_PFC_Init(PFC_HandleTypeDef* handle, const PFC_InitTypeDef* init) {
    if (handle == NULL || init == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    if (init->PLLInit == NULL || init->BusVoltagePIDInit == NULL ||
        init->GridCurrentPIDInit == NULL || init->Config == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    if (!VOLOOP_PFC_IsFiniteFloat(init->TriggerFrequency) || init->TriggerFrequency <= 0.0f ||
        !VOLOOP_PFC_IsConfigValid(init->Config)) {
        return VOLOOP_INVALID_PARAM;
    }
    if (!VOLOOP_PFC_IsFiniteFloat(init->PLLInit->triggerFrequency) ||
        init->PLLInit->triggerFrequency != init->TriggerFrequency) {
        return VOLOOP_INVALID_PARAM;
    }

    *handle = (PFC_HandleTypeDef){ 0 };
    handle->Config = *(init->Config);
    handle->TriggerFrequency = init->TriggerFrequency;
    handle->State = PFC_RESET;
    handle->FaultCode = PFC_FAULT_NONE;

    VOLOOP_StatusTypeDef status =
        VOLOOP_PID_Init(&(handle->BusVoltagePID), init->BusVoltagePIDInit);
    if (status != VOLOOP_OK) {
        VOLOOP_PFC_ClearHandle(handle);
        return status;
    }

    status = VOLOOP_PID_Init(&(handle->GridCurrentPID), init->GridCurrentPIDInit);
    if (status != VOLOOP_OK) {
        VOLOOP_PFC_ClearHandle(handle);
        return status;
    }

    status = VOLOOP_PLL_Init(&(handle->PLL), init->PLLInit);
    if (status != VOLOOP_OK) {
        VOLOOP_PFC_ClearHandle(handle);
        return status;
    }

    status = VOLOOP_PLL_Start(&(handle->PLL));
    if (status != VOLOOP_OK) {
        VOLOOP_PFC_ClearHandle(handle);
        return status;
    }

    VOLOOP_PFC_ResetRuntime(handle);
    handle->State = PFC_DISABLED;
    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_PFC_DeInit(PFC_HandleTypeDef* handle) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    VOLOOP_StatusTypeDef result = VOLOOP_OK;
    if (VOLOOP_PLL_GetState(&(handle->PLL)) == PLL_RUNNING) {
        result = VOLOOP_PLL_Stop(&(handle->PLL));
    }

    VOLOOP_StatusTypeDef status = VOLOOP_PLL_DeInit(&(handle->PLL));
    if (result == VOLOOP_OK && status != VOLOOP_OK) {
        result = status;
    }
    status = VOLOOP_PID_DeInit(&(handle->BusVoltagePID));
    if (result == VOLOOP_OK && status != VOLOOP_OK) {
        result = status;
    }
    status = VOLOOP_PID_DeInit(&(handle->GridCurrentPID));
    if (result == VOLOOP_OK && status != VOLOOP_OK) {
        result = status;
    }

    *handle = (PFC_HandleTypeDef){ 0 };
    return result;
}

VOLOOP_StatusTypeDef VOLOOP_PFC_Start(PFC_HandleTypeDef* handle) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    if (handle->State == PFC_WAIT_PLL || handle->State == PFC_RUNNING ||
        handle->State == PFC_CURRENT_LIMIT) {
        return VOLOOP_OK;
    }
    if (handle->State != PFC_DISABLED || handle->ReferenceConfigured == 0U) {
        return VOLOOP_INVALID_STATE;
    }
    if (VOLOOP_PLL_GetState(&(handle->PLL)) != PLL_RUNNING) {
        handle->State = PFC_ERROR;
        handle->FaultCode = PFC_FAULT_PLL;
        return VOLOOP_INVALID_STATE;
    }

    VOLOOP_StatusTypeDef status = VOLOOP_PFC_ResetControllers(handle);
    if (status != VOLOOP_OK) {
        return status;
    }

    VOLOOP_PFC_ResetRuntime(handle);
    handle->State = PFC_WAIT_PLL;
    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_PFC_Stop(PFC_HandleTypeDef* handle) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    if (handle->State == PFC_RESET || handle->State == PFC_ERROR) {
        return VOLOOP_INVALID_STATE;
    }

    VOLOOP_StatusTypeDef status = VOLOOP_PFC_ResetControllers(handle);
    if (status != VOLOOP_OK) {
        return status;
    }

    VOLOOP_PFC_ResetRuntime(handle);
    handle->State = PFC_DISABLED;
    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_PFC_ClearFaultCode(PFC_HandleTypeDef* handle) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    if (handle->State != PFC_ERROR && handle->State != PFC_DISABLED) {
        return VOLOOP_INVALID_STATE;
    }
    if (VOLOOP_PLL_GetState(&(handle->PLL)) == PLL_ERROR ||
        VOLOOP_PLL_GetState(&(handle->PLL)) == PLL_RESET) {
        return VOLOOP_INVALID_STATE;
    }

    if (handle->State == PFC_DISABLED && handle->FaultCode == PFC_FAULT_NONE) {
        VOLOOP_StatusTypeDef status = VOLOOP_PFC_ResetControllers(handle);
        if (status != VOLOOP_OK) {
            return status;
        }
        VOLOOP_PFC_ResetRuntime(handle);
        return VOLOOP_OK;
    }

    VOLOOP_StatusTypeDef status = VOLOOP_PLL_Reset(&(handle->PLL));
    if (status != VOLOOP_OK) {
        handle->State = PFC_ERROR;
        handle->FaultCode = PFC_FAULT_PLL;
        return status;
    }

    status = VOLOOP_PLL_Start(&(handle->PLL));
    if (status != VOLOOP_OK) {
        handle->State = PFC_ERROR;
        handle->FaultCode = PFC_FAULT_PLL;
        return status;
    }

    status = VOLOOP_PFC_ResetControllers(handle);
    if (status != VOLOOP_OK) {
        handle->State = PFC_ERROR;
        return status;
    }

    VOLOOP_PFC_ResetRuntime(handle);
    handle->FaultCode = PFC_FAULT_NONE;
    handle->State = PFC_DISABLED;
    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_PFC_SetValue(PFC_HandleTypeDef* handle, float busVoltage,
                                         float maxGridCurrent) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    if (handle->State == PFC_RESET || handle->State == PFC_ERROR) {
        return VOLOOP_INVALID_STATE;
    }
    if (!VOLOOP_PFC_IsFiniteFloat(busVoltage) || !VOLOOP_PFC_IsFiniteFloat(maxGridCurrent) ||
        busVoltage <= 0.0f || busVoltage >= handle->Config.BusOverVoltageThreshold ||
        maxGridCurrent <= 0.0f || maxGridCurrent >= handle->Config.GridOverCurrentThreshold) {
        return VOLOOP_INVALID_PARAM;
    }

    handle->TargetBusVoltage = busVoltage;
    handle->MaxGridCurrent = maxGridCurrent;
    handle->ReferenceConfigured = 1U;
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
        return PFC_FAULT_INVALID;
    }
    return handle->FaultCode;
}

PLL_LockStateTypeDef VOLOOP_PFC_GetPLLLockState(const PFC_HandleTypeDef* handle) {
    if (handle == NULL) {
        return PLL_UNLOCKED;
    }
    return VOLOOP_PLL_IsLocked(&(handle->PLL));
}

float VOLOOP_PFC_GetModulation(const PFC_HandleTypeDef* handle) {
    if (handle == NULL) {
        return 0.0f;
    }
    return handle->Modulation;
}

float VOLOOP_PFC_GetCurrentAmplitudeReference(const PFC_HandleTypeDef* handle) {
    if (handle == NULL) {
        return 0.0f;
    }
    return handle->CurrentAmplitudeReference;
}

float VOLOOP_PFC_GetCurrentReference(const PFC_HandleTypeDef* handle) {
    if (handle == NULL) {
        return 0.0f;
    }
    return handle->CurrentReference;
}

PFC_LinePolarityTypeDef VOLOOP_PFC_GetLinePolarity(const PFC_HandleTypeDef* handle) {
    if (handle == NULL) {
        return PFC_LINE_ZERO;
    }
    return handle->LinePolarity;
}

VOLOOP_StatusTypeDef VOLOOP_PFC_Sync(PFC_HandleTypeDef* handle, const PFC_InputTypeDef* input,
                                     PFC_OutputTypeDef* output) {
    if (output != NULL) {
        VOLOOP_PFC_DisableOutput(output);
    }
    if (handle == NULL || input == NULL || output == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    if (handle->State == PFC_RESET || handle->State == PFC_ERROR) {
        return VOLOOP_INVALID_STATE;
    }
    if (handle->State != PFC_DISABLED && handle->State != PFC_WAIT_PLL &&
        handle->State != PFC_RUNNING && handle->State != PFC_CURRENT_LIMIT) {
        return VOLOOP_INVALID_STATE;
    }

    if (!VOLOOP_PFC_IsFiniteFloat(input->GridVoltage)) {
        VOLOOP_PFC_EnterFault(handle, PFC_FAULT_INVALID_SAMPLE);
        return VOLOOP_ERROR;
    }

    if (handle->State != PFC_DISABLED) {
        if (!VOLOOP_PFC_IsFiniteFloat(input->GridCurrent) ||
            !VOLOOP_PFC_IsFiniteFloat(input->BusVoltage) || input->BusVoltage < 0.0f) {
            VOLOOP_PFC_EnterFault(handle, PFC_FAULT_INVALID_SAMPLE);
            return VOLOOP_ERROR;
        }
        if (fabsf(input->GridCurrent) > handle->Config.GridOverCurrentThreshold) {
            VOLOOP_PFC_EnterFault(handle, PFC_FAULT_INPUT_OVERCURRENT);
            return VOLOOP_ERROR;
        }
        if (input->BusVoltage > handle->Config.BusOverVoltageThreshold) {
            VOLOOP_PFC_EnterFault(handle, PFC_FAULT_BUS_OVERVOLTAGE);
            return VOLOOP_ERROR;
        }
    }

    VOLOOP_StatusTypeDef status = VOLOOP_PFC_EnsurePLLRunning(handle);
    if (status != VOLOOP_OK) {
        return status;
    }

    PLL_InputTypeDef pllInput = {
        .InputVoltage = input->GridVoltage,
    };
    status = VOLOOP_PLL_Sync(&(handle->PLL), &pllInput);
    if (status != VOLOOP_OK) {
        if (VOLOOP_PLL_GetState(&(handle->PLL)) == PLL_ERROR) {
            VOLOOP_PFC_EnterFault(handle, PFC_FAULT_PLL);
        }
        return status;
    }

    if (handle->State == PFC_DISABLED) {
        return VOLOOP_OK;
    }
    if (VOLOOP_PLL_IsLocked(&(handle->PLL)) != PLL_LOCKED) {
        (void)VOLOOP_PFC_ResetControllers(handle);
        VOLOOP_PFC_ResetRuntime(handle);
        handle->State = PFC_WAIT_PLL;
        return VOLOOP_OK;
    }

    int32_t phaseQ31 = VOLOOP_PLL_GetPhaseQ31(&(handle->PLL));
    float sine = VOLOOP_DEF_SIN(phaseQ31);
    PFC_LinePolarityTypeDef linePolarity =
        VOLOOP_PFC_GetLinePolarityFromSine(sine, handle->Config.ZeroCrossingDeadband);

    float currentAmplitudeReference =
        VOLOOP_PID_ComputeConditional(&(handle->BusVoltagePID), handle->TargetBusVoltage,
                                      input->BusVoltage, 0.0f, handle->MaxGridCurrent);
    handle->CurrentAmplitudeReference = currentAmplitudeReference;
    handle->CurrentReference = currentAmplitudeReference * sine;
    handle->LinePolarity = linePolarity;
    handle->State = (VOLOOP_PID_GetState(&(handle->BusVoltagePID)) == PID_UpperSaturated)
                        ? PFC_CURRENT_LIMIT
                        : PFC_RUNNING;

    if (linePolarity == PFC_LINE_ZERO) {
        (void)VOLOOP_PID_Reset(&(handle->GridCurrentPID));
        handle->Modulation = 0.0f;
        return VOLOOP_OK;
    }

    float currentReferenceMagnitude = currentAmplitudeReference * fabsf(sine);
    float currentMeasurementAligned =
        (linePolarity == PFC_LINE_POSITIVE) ? input->GridCurrent : -input->GridCurrent;
    float modulationMagnitude =
        VOLOOP_PID_ComputeConditional(&(handle->GridCurrentPID), currentReferenceMagnitude,
                                      currentMeasurementAligned, 0.0f, 1.0f);
    handle->Modulation =
        (linePolarity == PFC_LINE_POSITIVE) ? modulationMagnitude : -modulationMagnitude;

    float highFrequencyDuty =
        VOLOOP_DEF_ClampFloat(1.0f - modulationMagnitude, handle->Config.MinHighFrequencyDuty,
                              handle->Config.MaxHighFrequencyDuty);

    output->LeftLegPwmState = VOLOOP_PWM_ENABLE;
    output->RightLegPwmState = VOLOOP_PWM_ENABLE;
    if (linePolarity == PFC_LINE_POSITIVE) {
        output->LeftLegDuty = highFrequencyDuty;
        output->RightLegDuty = 0.0f;
    } else {
        output->LeftLegDuty = 0.0f;
        output->RightLegDuty = highFrequencyDuty;
    }

    return VOLOOP_OK;
}
