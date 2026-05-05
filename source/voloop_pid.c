#include "voloop_pid.h"
#include <stdlib.h>

/* ===== Static initializers (caller owns the memory) ===== */

VOLOOP_StatusTypeDef VOLOOP_PID_InitStaticDiscrete(PID_HandleTypeDef* handle, const PID_InitDiscreteTypeDef* init) {
    if (handle == NULL || init == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    handle->KpDiscrete = init->KpDiscrete;
    handle->KiDiscrete = init->KiDiscrete;
    handle->KdDiscrete = init->KdDiscrete;
    handle->Integral = 0.0f;
    handle->PreviousError = 0.0f;
    handle->State = PID_UnSaturated;
    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_PID_InitStaticContinue(PID_HandleTypeDef* handle, const PID_InitContinueTypeDef* init) {
    if (handle == NULL || init == NULL || init->triggerFrequency == 0U) {
        return VOLOOP_INVALID_PARAM;
    }
    handle->KpDiscrete = init->Kp;
    handle->KiDiscrete = init->Ki / init->triggerFrequency;
    handle->KdDiscrete = init->Kd * init->triggerFrequency;
    handle->Integral = 0.0f;
    handle->PreviousError = 0.0f;
    handle->State = PID_UnSaturated;
    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_PID_InitStaticOneZero(PID_HandleTypeDef* handle, const PID_InitOneZeroTypeDef* init) {
    if (handle == NULL || init == NULL || init->triggerFrequency == 0U) {
        return VOLOOP_INVALID_PARAM;
    }
    handle->KpDiscrete = init->gain;
    handle->KiDiscrete = VOLOOP_TwoPi * init->gain * init->zero / init->triggerFrequency;
    handle->KdDiscrete = 0.0f;
    handle->Integral = 0.0f;
    handle->PreviousError = 0.0f;
    handle->State = PID_UnSaturated;
    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_PID_InitStaticTwoZero(PID_HandleTypeDef* handle, const PID_InitTwoZeroTypeDef* init) {
    if (handle == NULL || init == NULL || init->triggerFrequency == 0U) {
        return VOLOOP_INVALID_PARAM;
    }
    handle->KpDiscrete = VOLOOP_TwoPi * init->gain * (init->zero1 + init->zero2);
    handle->KiDiscrete = VOLOOP_FourPiSquared * init->gain * init->zero1 * init->zero2 / init->triggerFrequency;
    handle->KdDiscrete = init->gain * init->triggerFrequency;
    handle->Integral = 0.0f;
    handle->PreviousError = 0.0f;
    handle->State = PID_UnSaturated;
    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_PID_InitStatic(PID_HandleTypeDef* handle, const PID_InitTypeDef* init) {
    if (handle == NULL || init == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    switch (init->mode) {
        case PID_Discrete:
            return VOLOOP_PID_InitStaticDiscrete(handle, &init->init.Discrete);
        case PID_Continue:
            return VOLOOP_PID_InitStaticContinue(handle, &init->init.Continue);
        case PID_OneZero:
            return VOLOOP_PID_InitStaticOneZero(handle, &init->init.OneZero);
        case PID_TwoZero:
            return VOLOOP_PID_InitStaticTwoZero(handle, &init->init.TwoZero);
        default:
            return VOLOOP_INVALID_PARAM;
    }
}

VOLOOP_StatusTypeDef VOLOOP_PID_DeInitStatic(PID_HandleTypeDef* handle) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    handle->Integral = 0.0f;
    handle->PreviousError = 0.0f;
    handle->State = PID_UnSaturated;
    return VOLOOP_OK;
}

/* ===== Dynamic initializers (malloc wrapper) ===== */

VOLOOP_StatusTypeDef VOLOOP_PID_InitDiscrete(PID_HandleTypeDef** handleOut, const PID_InitDiscreteTypeDef* init) {
    if (handleOut == NULL || *handleOut != NULL || init == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    PID_HandleTypeDef* handle = malloc(sizeof(PID_HandleTypeDef));
    if (handle == NULL) {
        return VOLOOP_BAD_ALLOCATE;
    }
    VOLOOP_StatusTypeDef status = VOLOOP_PID_InitStaticDiscrete(handle, init);
    if (status != VOLOOP_OK) {
        free(handle);
        return status;
    }
    *handleOut = handle;
    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_PID_InitContinue(PID_HandleTypeDef** handleOut, const PID_InitContinueTypeDef* init) {
    if (handleOut == NULL || *handleOut != NULL || init == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    PID_HandleTypeDef* handle = malloc(sizeof(PID_HandleTypeDef));
    if (handle == NULL) {
        return VOLOOP_BAD_ALLOCATE;
    }
    VOLOOP_StatusTypeDef status = VOLOOP_PID_InitStaticContinue(handle, init);
    if (status != VOLOOP_OK) {
        free(handle);
        return status;
    }
    *handleOut = handle;
    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_PID_InitOneZero(PID_HandleTypeDef** handleOut, const PID_InitOneZeroTypeDef* init) {
    if (handleOut == NULL || *handleOut != NULL || init == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    PID_HandleTypeDef* handle = malloc(sizeof(PID_HandleTypeDef));
    if (handle == NULL) {
        return VOLOOP_BAD_ALLOCATE;
    }
    VOLOOP_StatusTypeDef status = VOLOOP_PID_InitStaticOneZero(handle, init);
    if (status != VOLOOP_OK) {
        free(handle);
        return status;
    }
    *handleOut = handle;
    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_PID_InitTwoZero(PID_HandleTypeDef** handleOut, const PID_InitTwoZeroTypeDef* init) {
    if (handleOut == NULL || *handleOut != NULL || init == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    PID_HandleTypeDef* handle = malloc(sizeof(PID_HandleTypeDef));
    if (handle == NULL) {
        return VOLOOP_BAD_ALLOCATE;
    }
    VOLOOP_StatusTypeDef status = VOLOOP_PID_InitStaticTwoZero(handle, init);
    if (status != VOLOOP_OK) {
        free(handle);
        return status;
    }
    *handleOut = handle;
    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_PID_Init(PID_HandleTypeDef** handleOut, const PID_InitTypeDef* init) {
    if (handleOut == NULL || *handleOut != NULL || init == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    switch (init->mode) {
        case PID_Discrete:
            return VOLOOP_PID_InitDiscrete(handleOut, &init->init.Discrete);
        case PID_Continue:
            return VOLOOP_PID_InitContinue(handleOut, &init->init.Continue);
        case PID_OneZero:
            return VOLOOP_PID_InitOneZero(handleOut, &init->init.OneZero);
        case PID_TwoZero:
            return VOLOOP_PID_InitTwoZero(handleOut, &init->init.TwoZero);
        default:
            return VOLOOP_INVALID_PARAM;
    }
}

VOLOOP_StatusTypeDef VOLOOP_PID_DeInit(PID_HandleTypeDef** pHandle) {
    if (pHandle == NULL || *pHandle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    free(*pHandle);
    *pHandle = NULL;
    return VOLOOP_OK;
}

/* ===== Runtime operations ===== */

VOLOOP_StatusTypeDef VOLOOP_PID_Reset(PID_HandleTypeDef* handle) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    handle->Integral = 0.0f;
    handle->PreviousError = 0.0f;
    handle->State = PID_UnSaturated;
    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_PID_SetIntegral(PID_HandleTypeDef* handle, float integral) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    handle->Integral = integral;
    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_PID_SetPreviousError(PID_HandleTypeDef* handle, float previousError) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    handle->PreviousError = previousError;
    return VOLOOP_OK;
}

PID_StateTypeDef VOLOOP_PID_GetState(PID_HandleTypeDef* handle) {
    if (handle == NULL) {
        return PID_ERROR;
    }
    return handle->State;
}

float VOLOOP_PID_Compute(PID_HandleTypeDef* handle,
                         float setpoint,
                         float measurement) {
    if (handle == NULL) {
        return 0.0f;
    }
    float error = setpoint - measurement;
    float derivative = error - handle->PreviousError;
    handle->Integral += error;
    float output = (handle->KpDiscrete * error) + (handle->KiDiscrete * handle->Integral) + (handle->KdDiscrete * derivative);
    handle->PreviousError = error;
    handle->State = PID_UnSaturated;
    return output;
}

float VOLOOP_PID_ComputeConditional(PID_HandleTypeDef* handle,
                                    float setpoint,
                                    float measurement,
                                    float outputMin,
                                    float outputMax) {
    if (handle == NULL) {
        return 0.0f;
    }
    float error = setpoint - measurement;
    float newIntegral = handle->Integral + error;
    float derivative = error - handle->PreviousError;
    float output = (handle->KpDiscrete * error) + (handle->KiDiscrete * newIntegral) + (handle->KdDiscrete * derivative);

    if (output > outputMax) {
        output = outputMax;
    } else if (output < outputMin) {
        output = outputMin;
    }

    if (output >= outputMax && error > 0.0f) {
        newIntegral = handle->Integral;
        handle->State = PID_UpperSaturated;
    } else if (output <= outputMin && error < 0.0f) {
        newIntegral = handle->Integral;
        handle->State = PID_LowerSaturated;
    } else {
        handle->State = PID_UnSaturated;
    }
    handle->Integral = newIntegral;
    handle->PreviousError = error;

    return output;
}

float VOLOOP_PID_ComputeBackCalculation(PID_HandleTypeDef* handle,
                                        float setpoint,
                                        float measurement,
                                        float outputMin,
                                        float outputMax,
                                        float antiWindupGain) {
    if (handle == NULL) {
        return 0.0f;
    }

    float error = setpoint - measurement;
    float derivative = error - handle->PreviousError;
    float rawOutput = (handle->KpDiscrete * error) + (handle->KiDiscrete * handle->Integral) + (handle->KdDiscrete * derivative);
    float output = rawOutput;

    if (rawOutput > outputMax) {
        output = outputMax;
        handle->Integral += error + antiWindupGain * (outputMax - rawOutput);
        handle->State = PID_UpperSaturated;
    } else if (rawOutput < outputMin) {
        output = outputMin;
        handle->Integral += error + antiWindupGain * (outputMin - rawOutput);
        handle->State = PID_LowerSaturated;
    } else {
        handle->Integral += error;
        handle->State = PID_UnSaturated;
    }
    handle->PreviousError = error;

    return output;
}
