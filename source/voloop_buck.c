#include "voloop_buck.h"
#include <stdlib.h>

VOLOOP_StatusTypeDef VOLOOP_Buck_InitStatic(Buck_HandleTypeDef* handle, Buck_InitTypeDef* init) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    if (init == NULL
        || init->InitFunc == NULL
        || init->DeInitFunc == NULL
        || init->Start == NULL
        || init->Stop == NULL
        || init->SetDuty == NULL
        || init->GetOutputVoltage == NULL
        || init->GetInductorCurrent == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    handle->Init = *init;
    handle->TargetOutputVoltage = 0.0f;
    handle->MaxInductorCurrent = 0.0f;
    handle->OverVoltageThreshold = init->OverVoltageThreshold;
    handle->OverCurrentThreshold = init->OverCurrentThreshold;
    handle->Duty = 0.0f;
    handle->State = BUCK_DISABLED;
    handle->FaultCode = BUCK_NOERROR;

    init->InitFunc();

    VOLOOP_StatusTypeDef status;
    status = VOLOOP_PID_InitStatic(&handle->OutPutVoltagePID, init->OutPutVoltagePIDInit);
    if (status != VOLOOP_OK) {
        return status;
    }
    status = VOLOOP_PID_InitStatic(&handle->InductorCurrentPID, init->InductorCurrentPIDInit);
    if (status != VOLOOP_OK) {
        VOLOOP_PID_DeInitStatic(&handle->OutPutVoltagePID);
        return status;
    }

    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_Buck_Init(Buck_HandleTypeDef** handleOut, Buck_InitTypeDef* init) {
    if (handleOut == NULL || *handleOut != NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    Buck_HandleTypeDef* handle = malloc(sizeof(Buck_HandleTypeDef));
    if (handle == NULL) {
        return VOLOOP_BAD_ALLOCATE;
    }

    VOLOOP_StatusTypeDef status = VOLOOP_Buck_InitStatic(handle, init);
    if (status != VOLOOP_OK) {
        free(handle);
        return status;
    }
    *handleOut = handle;
    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_Buck_DeInitStatic(Buck_HandleTypeDef* handle) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    handle->Init.Stop();
    handle->Init.DeInitFunc();

    VOLOOP_PID_DeInitStatic(&handle->OutPutVoltagePID);
    VOLOOP_PID_DeInitStatic(&handle->InductorCurrentPID);

    handle->State = BUCK_DISABLED;
    handle->FaultCode = BUCK_NOERROR;
    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_Buck_DeInit(Buck_HandleTypeDef** pHandle) {
    if (pHandle == NULL || *pHandle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    VOLOOP_Buck_DeInitStatic(*pHandle);
    free(*pHandle);
    *pHandle = NULL;
    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_Buck_Start(Buck_HandleTypeDef* handle) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    if (handle->State == BUCK_ERROR
        || handle->State == BUCK_CVMODE
        || handle->State == BUCK_CCMODE) {
        return VOLOOP_INVALID_STATE;
    }

    VOLOOP_PID_Reset(&handle->OutPutVoltagePID);
    VOLOOP_PID_Reset(&handle->InductorCurrentPID);
    handle->State = BUCK_CVMODE;
    handle->Init.Start();

    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_Buck_Stop(Buck_HandleTypeDef* handle) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    handle->Init.Stop();
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
    if (handle->State != BUCK_ERROR
        && handle->State != BUCK_DISABLED) {
        return VOLOOP_INVALID_STATE;
    }
    handle->FaultCode = BUCK_NOERROR;
    handle->State = BUCK_DISABLED;
    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_Buck_SetValue(Buck_HandleTypeDef* handle, float voltage, float current) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    handle->TargetOutputVoltage = voltage;
    handle->MaxInductorCurrent = current;
    return VOLOOP_OK;
}

float VOLOOP_Buck_GetDuty(Buck_HandleTypeDef* handle) {
    if (handle == NULL) {
        return 0.0f;
    }
    return handle->Duty;
}

VOLOOP_StatusTypeDef VOLOOP_Buck_Sync(Buck_HandleTypeDef* handle) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    if (handle->State == BUCK_ERROR) {
        return VOLOOP_INVALID_STATE;
    } else if (handle->State == BUCK_DISABLED) {
        return VOLOOP_INVALID_STATE;
    }

    float presentVoltage = handle->Init.GetOutputVoltage();
    float presentCurrent = handle->Init.GetInductorCurrent();

    if (presentCurrent > handle->OverCurrentThreshold) {
        VOLOOP_Buck_Stop(handle);
        handle->State = BUCK_ERROR;
        handle->FaultCode = BUCK_OCP;
        return VOLOOP_ERROR;
    }
    if (presentVoltage > handle->OverVoltageThreshold) {
        VOLOOP_Buck_Stop(handle);
        handle->State = BUCK_ERROR;
        handle->FaultCode = BUCK_OVP;
        return VOLOOP_ERROR;
    }

    float targetInductorCurrent = VOLOOP_PID_ComputeConditional(&handle->OutPutVoltagePID, handle->TargetOutputVoltage, presentVoltage, 0.0f, handle->MaxInductorCurrent);

    if (VOLOOP_PID_GetState(&handle->OutPutVoltagePID) == PID_UpperSaturated) {
        handle->State = BUCK_CCMODE;
    } else {
        handle->State = BUCK_CVMODE;
    }

    float duty = VOLOOP_PID_ComputeConditional(&handle->InductorCurrentPID, targetInductorCurrent, presentCurrent, BUCK_MIN_DUTY, BUCK_MAX_DUTY);

    handle->Duty = duty;
    handle->Init.SetDuty(duty);

    return VOLOOP_OK;
}
