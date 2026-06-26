#include "voloop_offinv.h"

static void VOLOOP_OffInv_DisableOutput(OffInv_OutputTypeDef* output) {
    output->LeftLegPwmState = VOLOOP_PWM_DISABLED;
    output->RightLegPwmState = VOLOOP_PWM_DISABLED;
    output->LeftLegDuty = 0.0f;
    output->RightLegDuty = 0.0f;
}

static inline int VOLOOP_OffInv_IsFiniteFloat(float value) {
    return isfinite(value);
}

VOLOOP_StatusTypeDef VOLOOP_OffInv_Init(OffInv_HandleTypeDef* handle, OffInv_InitTypeDef* init) {
    if (handle == NULL || init == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    if (init->VoltageQPRInit == NULL || init->NCOInit == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    if (!VOLOOP_OffInv_IsFiniteFloat(init->triggerFrequency) || init->triggerFrequency <= 0.0f) {
        return VOLOOP_INVALID_PARAM;
    }

    *handle = (OffInv_HandleTypeDef){ 0 };
    handle->NominalFrequency = init->NCOInit->initialFrequency;
    handle->triggerFrequency = init->triggerFrequency;
    handle->TargetVoltage = 0.0f;
    handle->VoltageQPRKb = OFFINV_DEFAULT_QPR_KB;
    handle->State = OFFINV_DISABLED;
    handle->FaultCode = OFFINV_NOERROR;
    handle->Duty = 0.0f;
    handle->VoltageQPR = (QPR_HandleTypeDef){ 0 };
    handle->NCO = (NCO_HandleTypeDef){ 0 };

    VOLOOP_StatusTypeDef status;
    status = VOLOOP_QPR_Init(&(handle->VoltageQPR), init->VoltageQPRInit);
    if (status != VOLOOP_OK) {
        VOLOOP_OffInv_DeInit(handle);
        return status;
    }

    status = VOLOOP_NCO_Init(&(handle->NCO), init->NCOInit);
    if (status != VOLOOP_OK) {
        VOLOOP_OffInv_DeInit(handle);
        return status;
    }

    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_OffInv_DeInit(OffInv_HandleTypeDef* handle) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    VOLOOP_QPR_DeInit(&(handle->VoltageQPR));
    VOLOOP_NCO_DeInit(&(handle->NCO));

    *handle = (OffInv_HandleTypeDef){ 0 };
    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_OffInv_Start(OffInv_HandleTypeDef* handle) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    if (handle->State == OFFINV_RUNNING) {
        return VOLOOP_OK;
    } else if (handle->State != OFFINV_DISABLED) {
        return VOLOOP_INVALID_STATE;
    }

    VOLOOP_StatusTypeDef status = VOLOOP_NCO_Start(&(handle->NCO));
    if (status != VOLOOP_OK) {
        handle->State = OFFINV_ERROR;
        handle->FaultCode = OFFINV_NCO;
        return status;
    }

    VOLOOP_QPR_Reset(&(handle->VoltageQPR));
    handle->State = OFFINV_RUNNING;
    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_OffInv_Stop(OffInv_HandleTypeDef* handle) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    if (handle->State == OFFINV_DISABLED) {
        return VOLOOP_OK;
    } else if (handle->State != OFFINV_RUNNING) {
        return VOLOOP_INVALID_STATE;
    }

    VOLOOP_StatusTypeDef status = VOLOOP_NCO_Stop(&(handle->NCO));
    if (status != VOLOOP_OK) {
        handle->State = OFFINV_ERROR;
        handle->FaultCode = OFFINV_NCO;
        return status;
    }

    handle->State = OFFINV_DISABLED;
    return VOLOOP_OK;
}

OffInv_StateTypeDef VOLOOP_OffInv_GetState(OffInv_HandleTypeDef* handle) {
    if (handle == NULL) {
        return OFFINV_ERROR;
    }
    return handle->State;
}

OffInv_FaultCodeTypeDef VOLOOP_OffInv_GetFaultCode(OffInv_HandleTypeDef* handle) {
    if (handle == NULL) {
        return OFFINV_INVALID;
    }
    return handle->FaultCode;
}

VOLOOP_StatusTypeDef VOLOOP_OffInv_ClearFaultCode(OffInv_HandleTypeDef* handle) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    if (handle->State != OFFINV_ERROR && handle->State != OFFINV_DISABLED) {
        return VOLOOP_INVALID_STATE;
    }
    if (VOLOOP_NCO_GetState(&(handle->NCO)) == NCO_ERROR) {
        VOLOOP_StatusTypeDef status = VOLOOP_NCO_ClearFaultCode(&(handle->NCO));
        if (status != VOLOOP_OK) {
            return status;
        }
    }
    handle->FaultCode = OFFINV_NOERROR;
    handle->State = OFFINV_DISABLED;
    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_OffInv_SetValue(OffInv_HandleTypeDef* handle, float PeakVoltage) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    if (!VOLOOP_OffInv_IsFiniteFloat(PeakVoltage) || PeakVoltage > OFFINV_OVTHRESHOLD) {
        return VOLOOP_INVALID_PARAM;
    }
    handle->TargetVoltage = PeakVoltage;
    return VOLOOP_OK;
}

float VOLOOP_OffInv_GetDuty(OffInv_HandleTypeDef* handle) {
    if (handle == NULL) {
        return 0.0f;
    }
    return handle->Duty;
}

VOLOOP_StatusTypeDef VOLOOP_OffInv_Sync(OffInv_HandleTypeDef* handle,
                                        const OffInv_InputTypeDef* input,
                                        OffInv_OutputTypeDef* output) {
    if (handle == NULL || input == NULL || output == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    if (handle->State == OFFINV_ERROR) {
        VOLOOP_OffInv_DisableOutput(output);
        return VOLOOP_INVALID_STATE;
    } else if (handle->State == OFFINV_DISABLED) {
        VOLOOP_OffInv_DisableOutput(output);
        return VOLOOP_OK;
    }

    // Protection: overcurrent
    if (fabsf(input->OutputCurrent) > OFFINV_OCTHRESHOLD) {
        handle->State = OFFINV_ERROR;
        handle->FaultCode = OFFINV_OCP;
        VOLOOP_OffInv_DisableOutput(output);
        return VOLOOP_ERROR;
    }
    // Protection: overvoltage
    if (fabsf(input->OutputVoltage) > OFFINV_OVTHRESHOLD) {
        handle->State = OFFINV_ERROR;
        handle->FaultCode = OFFINV_OVP;
        VOLOOP_OffInv_DisableOutput(output);
        return VOLOOP_ERROR;
    }

    // NCO sync: advance phase
    VOLOOP_StatusTypeDef status = VOLOOP_NCO_Sync(&(handle->NCO));
    if (status != VOLOOP_OK) {
        handle->State = OFFINV_ERROR;
        handle->FaultCode = OFFINV_NCO;
        VOLOOP_OffInv_DisableOutput(output);
        return status;
    }

    // Reference voltage: Vref = TargetVoltage * sin(NCO_phase)
    float sinRef = VOLOOP_NCO_GetSine(&(handle->NCO));
    float vref = handle->TargetVoltage * sinRef;

    // QPR voltage control: error = Vref - Vout
    float error = vref - input->OutputVoltage;
    float modulation = VOLOOP_QPR_ComputeBackCalculation(
        &(handle->VoltageQPR), error, -OFFINV_MAX_DUTY, OFFINV_MAX_DUTY, handle->VoltageQPRKb);

    handle->Duty = modulation;

    // Unipolar SPWM: low-side MOSFET allowed constant-on (duty=0)
    if (sinRef >= 0.0f) {
        // Positive half-cycle: left leg SPWM, right leg low-side on
        output->LeftLegPwmState = VOLOOP_PWM_ENABLE;
        output->LeftLegDuty = VOLOOP_DEF_ClampFloat(modulation, 0.0f, OFFINV_MAX_DUTY);
        output->RightLegPwmState = VOLOOP_PWM_ENABLE;
        output->RightLegDuty = 0.0f;
    } else {
        // Negative half-cycle: right leg SPWM, left leg low-side on
        output->LeftLegPwmState = VOLOOP_PWM_ENABLE;
        output->LeftLegDuty = 0.0f;
        output->RightLegPwmState = VOLOOP_PWM_ENABLE;
        output->RightLegDuty = VOLOOP_DEF_ClampFloat(-modulation, 0.0f, OFFINV_MAX_DUTY);
    }

    return VOLOOP_OK;
}
