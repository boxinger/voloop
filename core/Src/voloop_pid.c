#include "voloop_pid.h"

VOLOOP_StatusTypeDef VOLOOP_PID_Init(PID_HandleTypeDef* handle, const PID_InitTypeDef* init) {
    if (handle == NULL || init == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    switch (init->mode) {
        case PID_Discrete:
            return VOLOOP_PID_InitDiscrete(handle, &init->init.Discrete);
        case PID_Continue:
            return VOLOOP_PID_InitContinue(handle, &init->init.Continue);
        case PID_OneZero:
            return VOLOOP_PID_InitOneZero(handle, &init->init.OneZero);
        case PID_TwoZero:
            return VOLOOP_PID_InitTwoZero(handle, &init->init.TwoZero);
        default:
            return VOLOOP_INVALID_PARAM;
    }
}

VOLOOP_StatusTypeDef VOLOOP_PID_InitDiscrete(PID_HandleTypeDef* handle,
                                             const PID_InitDiscreteTypeDef* init) {
    // Verify input parameter
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

VOLOOP_StatusTypeDef VOLOOP_PID_InitContinue(PID_HandleTypeDef* handle,
                                             const PID_InitContinueTypeDef* init) {
    // Verify input parameter
    if (handle == NULL || init == NULL || init->triggerFrequency == 0U) {
        return VOLOOP_INVALID_PARAM;
    }

    // handle->Init = *init;
    handle->KpDiscrete = init->Kp;
    handle->KiDiscrete = init->Ki / init->triggerFrequency;
    handle->KdDiscrete = init->Kd * init->triggerFrequency;
    handle->Integral = 0.0f;
    handle->PreviousError = 0.0f;
    handle->State = PID_UnSaturated;

    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_PID_InitOneZero(PID_HandleTypeDef* handle,
                                            const PID_InitOneZeroTypeDef* init) {
    // Verify input parameter
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    if (init == NULL || init->triggerFrequency == 0U) {
        return VOLOOP_INVALID_PARAM;
    }

    // handle->Init = *init;
    handle->KpDiscrete = init->gain;
    handle->KiDiscrete = VOLOOP_TwoPi * init->gain * init->zero / init->triggerFrequency;
    handle->KdDiscrete = 0.0f;
    handle->Integral = 0.0f;
    handle->PreviousError = 0.0f;
    handle->State = PID_UnSaturated;

    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_PID_InitTwoZero(PID_HandleTypeDef* handle,
                                            const PID_InitTwoZeroTypeDef* init) {
    // Verify input parameter
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    if (init == NULL || init->triggerFrequency == 0U) {
        return VOLOOP_INVALID_PARAM;
    }

    // handle->Init = *init;
    handle->KpDiscrete = VOLOOP_TwoPi * init->gain * (init->zero1 + init->zero2);
    handle->KiDiscrete =
        VOLOOP_FourPiSquared * init->gain * init->zero1 * init->zero2 / init->triggerFrequency;
    handle->KdDiscrete = init->gain * init->triggerFrequency;
    handle->Integral = 0.0f;
    handle->PreviousError = 0.0f;
    handle->State = PID_UnSaturated;

    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_PID_DeInit(PID_HandleTypeDef* handle) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    handle->KpDiscrete = 0.0f;
    handle->KiDiscrete = 0.0f;
    handle->KdDiscrete = 0.0f;
    handle->Integral = 0.0f;
    handle->PreviousError = 0.0f;
    handle->State = PID_ERROR;

    return VOLOOP_OK;
}

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

float VOLOOP_PID_Compute(PID_HandleTypeDef* handle, float setpoint, float measurement) {
    if (handle == NULL) {
        return 0.0f;
    }
    float error = setpoint - measurement;
    float derivative = error - handle->PreviousError;
    handle->Integral += error;
    float output = (handle->KpDiscrete * error) + (handle->KiDiscrete * handle->Integral) +
                   (handle->KdDiscrete * derivative);
    handle->PreviousError = error;
    handle->State = PID_UnSaturated;
    return output;
}

float VOLOOP_PID_ComputeConditional(PID_HandleTypeDef* handle, float setpoint, float measurement,
                                    float outputMin, float outputMax) {
    if (handle == NULL) {
        return 0.0f;
    }
    float error = setpoint - measurement;
    float derivative = error - handle->PreviousError;

    // 1) Tentatively integrate, then compute the *raw* (pre-clamp) output.
    float newIntegral = handle->Integral + error;
    float rawOutput = (handle->KpDiscrete * error) + (handle->KiDiscrete * newIntegral) +
                      (handle->KdDiscrete * derivative);

    // 2) Conditional integration (anti-windup): decide on the raw output,
    //    *before* clamping. Freeze the integral only when the output is
    //    saturated AND the error would drive it further into saturation.
    if (rawOutput > outputMax && error > 0.0f) {
        newIntegral = handle->Integral; // hold integral, no windup
        handle->State = PID_UpperSaturated;
    } else if (rawOutput < outputMin && error < 0.0f) {
        newIntegral = handle->Integral; // hold integral, no windup
        handle->State = PID_LowerSaturated;
    } else {
        handle->State = PID_UnSaturated; // free to integrate
    }
    handle->Integral = newIntegral;
    handle->PreviousError = error;

    // 3) Strict output limiting: the returned value is always within range.
    if (rawOutput > outputMax) {
        return outputMax;
    } else if (rawOutput < outputMin) {
        return outputMin;
    }
    return rawOutput;
}

float VOLOOP_PID_ComputeBackCalculation(PID_HandleTypeDef* handle, float setpoint,
                                        float measurement, float outputMin, float outputMax,
                                        float Kb) {
    if (handle == NULL) {
        return 0.0f;
    }

    float error = setpoint - measurement;
    float derivative = error - handle->PreviousError;
    float rawOutput = (handle->KpDiscrete * error) + (handle->KiDiscrete * handle->Integral) +
                      (handle->KdDiscrete * derivative);
    float output = rawOutput;

    // Anti-windup back-calculation
    if (rawOutput > outputMax) {
        output = outputMax;
        handle->Integral += error + Kb * (outputMax - rawOutput);
        handle->State = PID_UpperSaturated;
    } else if (rawOutput < outputMin) {
        output = outputMin;
        handle->Integral += error + Kb * (outputMin - rawOutput);
        handle->State = PID_LowerSaturated;
    } else {
        handle->Integral += error;
        handle->State = PID_UnSaturated;
    }
    handle->PreviousError = error;

    return output;
}
