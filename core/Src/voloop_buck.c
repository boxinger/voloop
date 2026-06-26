#include "voloop_buck.h"

static void VOLOOP_Buck_DisableOutput(Buck_OutputTypeDef* output) {
    output->PwmState = VOLOOP_PWM_DISABLED;
    output->Duty = 0.0f;
}

VOLOOP_StatusTypeDef VOLOOP_Buck_Init(Buck_HandleTypeDef* handle, Buck_InitTypeDef* init) {
    // Verify input parameters
    if (handle == NULL || init == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    if (init->OutPutVoltagePIDInit == NULL || init->InductorCurrentPIDInit == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    // Load initialization parameters
    *handle = (Buck_HandleTypeDef){ 0 };
    handle->Init = *init;
    handle->TargetOutputVoltage = 0.0f;
    handle->MaxInductorCurrent = 0.0f;
    handle->Duty = 0.0f;
    handle->State = BUCK_DISABLED;
    handle->FaultCode = BUCK_NOERROR;
    handle->OutPutVoltagePID = (PID_HandleTypeDef){ 0 };
    handle->InductorCurrentPID = (PID_HandleTypeDef){ 0 };

    //PID initialization
    VOLOOP_StatusTypeDef status;
    status = VOLOOP_PID_Init(&(handle->OutPutVoltagePID), init->OutPutVoltagePIDInit);
    if (status != VOLOOP_OK) {
        VOLOOP_Buck_DeInit(handle);
        return status;
    }
    status = VOLOOP_PID_Init(&(handle->InductorCurrentPID), init->InductorCurrentPIDInit);
    if (status != VOLOOP_OK) {
        VOLOOP_Buck_DeInit(handle);
        return status;
    }

    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_Buck_DeInit(Buck_HandleTypeDef* handle) {
    // Verify input parameter
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    // Deinitialize PID controllers
    VOLOOP_PID_DeInit(&(handle->OutPutVoltagePID));
    VOLOOP_PID_DeInit(&(handle->InductorCurrentPID));

    // Free Buck handle memory
    *handle = (Buck_HandleTypeDef){ 0 };
    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_Buck_Start(Buck_HandleTypeDef* handle) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    if (handle->State == BUCK_ERROR || handle->State == BUCK_CVMODE ||
        handle->State == BUCK_CCMODE) {
        return VOLOOP_INVALID_STATE;
    }

    VOLOOP_PID_Reset(&(handle->OutPutVoltagePID));
    VOLOOP_PID_Reset(&(handle->InductorCurrentPID));
    handle->Duty = 0.0f;
    handle->State = BUCK_CVMODE; // Default to CV mode when starting

    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_Buck_Stop(Buck_HandleTypeDef* handle) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    if (handle->State == BUCK_DISABLED) {
        return VOLOOP_OK;
    }

    if (handle->State == BUCK_ERROR) {
        return VOLOOP_INVALID_STATE;
    }

    handle->State = BUCK_DISABLED;
    return VOLOOP_OK;
}

Buck_StateTypeDef VOLOOP_Buck_GetState(Buck_HandleTypeDef* handle) {
    if (handle == NULL) {
        return BUCK_ERROR;
    }
    return handle->State;
}

Buck_FaultCodeTypeDef VOLOOP_Buck_GetFaultCode(Buck_HandleTypeDef* handle) {
    if (handle == NULL) {
        return BUCK_INVALID;
    }
    return handle->FaultCode;
}

VOLOOP_StatusTypeDef VOLOOP_Buck_ClearFaultCode(Buck_HandleTypeDef* handle) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    if (handle->State != BUCK_ERROR && handle->State != BUCK_DISABLED) {
        return VOLOOP_INVALID_STATE;
    }
    handle->FaultCode = BUCK_NOERROR;
    handle->State = BUCK_DISABLED;
    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_Buck_SetValue(Buck_HandleTypeDef* handle, float Voltage,
                                          float Current) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    handle->TargetOutputVoltage = Voltage;
    handle->MaxInductorCurrent = Current;
    return VOLOOP_OK;
}

float VOLOOP_Buck_GetDuty(Buck_HandleTypeDef* handle) {
    if (handle == NULL) {
        return 0.0f;
    }
    return handle->Duty;
}

VOLOOP_StatusTypeDef VOLOOP_Buck_Sync(Buck_HandleTypeDef* handle, const Buck_InputTypeDef* input,
                                      Buck_OutputTypeDef* output) {
    // Verify input parameter
    if (handle == NULL || input == NULL || output == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    if (handle->State == BUCK_ERROR) {
        VOLOOP_Buck_DisableOutput(output);
        return VOLOOP_INVALID_STATE;
    } else if (handle->State == BUCK_DISABLED) {
        VOLOOP_Buck_DisableOutput(output);
        return VOLOOP_OK;
    }

    //Get necessary parameters
    float presentVoltage = input->OutputVoltage;
    float presentCurrent = input->InductorCurrent;

    //Protection
    if (presentCurrent > BUCK_OCTHRESHOLD) {
        handle->State = BUCK_ERROR;
        handle->FaultCode = BUCK_OCP;
        VOLOOP_Buck_DisableOutput(output);
        return VOLOOP_ERROR;
    }
    if (presentVoltage > BUCK_OVTHRESHOLD) {
        handle->State = BUCK_ERROR;
        handle->FaultCode = BUCK_OVP;
        VOLOOP_Buck_DisableOutput(output);
        return VOLOOP_ERROR;
    }

    //Compute target inductor current based on output voltage error, with anti-windup
    float targetInductorCurrent =
        VOLOOP_PID_ComputeConditional(&(handle->OutPutVoltagePID), handle->TargetOutputVoltage,
                                      presentVoltage, 0.0f, handle->MaxInductorCurrent);

    if (VOLOOP_PID_GetState(&(handle->OutPutVoltagePID)) == PID_UpperSaturated) {
        handle->State = BUCK_CCMODE;
    } else {
        handle->State = BUCK_CVMODE;
    }

    float duty = VOLOOP_PID_ComputeConditional(&(handle->InductorCurrentPID), targetInductorCurrent,
                                               presentCurrent, BUCK_MIN_DUTY, BUCK_MAX_DUTY);

    // Set duty cycle
    handle->Duty = duty;
    output->PwmState = VOLOOP_PWM_ENABLE;
    output->Duty = duty;

    return VOLOOP_OK;
}
