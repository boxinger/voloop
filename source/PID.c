#include "PID.h"
#include <stdlib.h>

struct PID_HandleTypeDef {
    // PID_InitTypeDef Init;
    float KpDiscrete;
    float KiDiscrete;
    float KdDiscrete;
    float Integral;
    float PreviousError;
    PID_StateTypeDef State;
};


PID_HandleTypeDef* PID_InitDiscrete(PID_InitDiscreteTypeDef* init) {
    // Verify input parameter
    if (init == NULL) {
        return NULL;
    }

    // Allocate memory for PID handle
    PID_HandleTypeDef* handle = (PID_HandleTypeDef*)malloc(sizeof(PID_HandleTypeDef));
    if (handle == NULL) {
        return NULL;
    }

    // handle->Init = *init;
    handle->KpDiscrete = init->KpDiscrete;
    handle->KiDiscrete = init->KiDiscrete;
    handle->KdDiscrete = init->KdDiscrete;
    handle->Integral = 0.0f;
    handle->PreviousError = 0.0f;
    handle->State = PID_UnSaturated;

    return handle;
}

    
PID_HandleTypeDef* PID_InitContinue(PID_InitContinueTypeDef* init) {
    // Verify input parameter
    if (init == NULL || init->triggerFrequency == 0U) {
        return NULL;
    }

    // Allocate memory for PID handle
    PID_HandleTypeDef* handle = (PID_HandleTypeDef*)malloc(sizeof(PID_HandleTypeDef));
    if (handle == NULL) {
        return NULL;
    }

    // handle->Init = *init;
    handle->KpDiscrete = init->Kp;
    handle->KiDiscrete = init->Ki / init->triggerFrequency;
    handle->KdDiscrete = init->Kd * init->triggerFrequency;
    handle->Integral = 0.0f;
    handle->PreviousError = 0.0f;
    handle->State = PID_UnSaturated;

    return handle;
}


PID_HandleTypeDef* PID_Init1Zero(PID_Init1ZeroTypeDef* init) {
    // Verify input parameter
    if (init == NULL || init->triggerFrequency == 0U) {
        return NULL;
    }

    // Allocate memory for PID handle
    PID_HandleTypeDef* handle = (PID_HandleTypeDef*)malloc(sizeof(PID_HandleTypeDef));
    if (handle == NULL) {
        return NULL;
    }

    // handle->Init = *init;
    handle->KpDiscrete = init->gain;
    handle->KiDiscrete = PID_TwoPi * init->gain * init->zero / init->triggerFrequency; 
    handle->KdDiscrete = 0.0f; 
    handle->Integral = 0.0f;
    handle->PreviousError = 0.0f;
    handle->State = PID_UnSaturated;

    return handle;
}


PID_HandleTypeDef* PID_Init2Zero(PID_Init2ZeroTypeDef* init) {
    // Verify input parameter
    if (init == NULL || init->triggerFrequency == 0U) {
        return NULL;
    }

    // Allocate memory for PID handle
    PID_HandleTypeDef* handle = (PID_HandleTypeDef*)malloc(sizeof(PID_HandleTypeDef));
    if (handle == NULL) {
        return NULL;
    }

    // handle->Init = *init;
    handle->KpDiscrete = PID_TwoPi * init->gain * (init->zero1 + init->zero2);
    handle->KiDiscrete = PID_FourPiSquared * init->gain * init->zero1 * init->zero2 / init->triggerFrequency;
    handle->KdDiscrete = init->gain * init->triggerFrequency;
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
    float output = (handle->KpDiscrete * error) + (handle->KiDiscrete * handle->Integral) + (handle->KdDiscrete * derivative);
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
	float output = (handle->KpDiscrete * error) + (handle->KiDiscrete * newIntegral) + (handle->KdDiscrete * derivative);

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
    float rawOutput = (handle->KpDiscrete * error) + (handle->KiDiscrete * handle->Integral) + (handle->KdDiscrete * derivative);
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
