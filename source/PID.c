#include "PID.h"
#include <stdlib.h>

struct PID_HandleTypeDef {
    PID_InitTypeDef Init;
    float Integral;
    float PreviousError;
    PID_StateTypeDef State;
};

// static float PID_Clamp(float value, float min, float max) {
//     if (value > max) {
//         return max;
//     }
//     if (value < min) {
//         return min;
//     }
//     return value;
// }

PID_HandleTypeDef* PID_Init(PID_InitTypeDef* init) {
    // Verify input parameter
    if (init == NULL) {
        return NULL;
    }

    // Allocate memory for PID handle
    PID_HandleTypeDef* handle = (PID_HandleTypeDef*)malloc(sizeof(PID_HandleTypeDef));
    if (handle == NULL) {
        return NULL;
    }

    handle->Init = *init;
    handle->Integral = 0.0f;
    handle->PreviousError = 0.0f;
    handle->State = PID_UnSaturated;

    return handle;
}

void PID_DeInit(PID_HandleTypeDef* handle) {
    if (handle == NULL) {
        return;
    }
    free(handle);
}

void PID_Reset(PID_HandleTypeDef* handle) {
    if (handle == NULL) {
        return;
    }
    handle->Integral = 0.0f;
    handle->PreviousError = 0.0f;
    handle->State = PID_UnSaturated;
}

void PID_SetIntegral(PID_HandleTypeDef* handle, float integral) {
	if (handle == NULL) {
		return;
	}
	handle->Integral = integral;
}

void PID_SetPreviousError(PID_HandleTypeDef* handle, float previousError) {
	if (handle == NULL) {
		return;
	}
	handle->PreviousError = previousError;
}

PID_StateTypeDef PID_GetState(PID_HandleTypeDef* handle) {
    if (handle == NULL) {
        return PID_ERROR;
    }
    return handle->State;
}

float PID_Compute(PID_HandleTypeDef* handle, 
							float setpoint, 
							float measurement) {
	if (handle == NULL) {
		return 0.0f;
	}
	float error = setpoint - measurement;
    float derivative = error - handle->PreviousError;
	handle->Integral += error;
    float output = (handle->Init.Kp * error) + (handle->Init.Ki * handle->Integral) + (handle->Init.Kd * derivative);
	handle->PreviousError = error;
    handle->State = PID_UnSaturated; 
	return output;
}

float PID_ComputeConditional(PID_HandleTypeDef* handle, 
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
	float output = (handle->Init.Kp * error) + (handle->Init.Ki * newIntegral) + (handle->Init.Kd * derivative);

    if (output > outputMax){
        output = outputMax; 
    } else if (output < outputMin) {
        output = outputMin;
    }

    // Anti-windup
    if (output >= outputMax && error > 0.0f){
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

float PID_ComputeBackCalculation(PID_HandleTypeDef* handle,
                                float setpoint,
                                float measurement,
                                float outputMin,
                                float outputMax,
                                float antiWindupGain){
    if (handle == NULL) {
        return 0.0f;
    }

    float error = setpoint - measurement;
    float derivative = error - handle->PreviousError;
    float rawOutput = (handle->Init.Kp * error) + (handle->Init.Ki * handle->Integral) + (handle->Init.Kd * derivative);
    float output = rawOutput;

    // Anti-windup back-calculation
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
