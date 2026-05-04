#include "buck.h"
#include <stdlib.h>

struct Buck_HandleTypeDef {
    Buck_InitTypeDef Init;
    PID_HandleTypeDef* OutPutVoltagePID;
    PID_HandleTypeDef* InductorCurrentPID;
    Buck_StateTypeDef State;
    Buck_FaultCodeTypeDef FaultCode;
    float TargetOutputVoltage;
    float MaxInductorCurrent;
    float Duty;
};

Buck_HandleTypeDef*  VOLOOP_Buck_Init(Buck_InitTypeDef* init) {
    // Verify input parameters
    if (init == NULL 
        || init->InitFunc == NULL
        || init->DeInitFunc == NULL
        || init->Start == NULL
        || init->Stop == NULL
        || init->SetDuty == NULL
        || init->GetOutputVoltage == NULL
        || init->GetInductorCurrent == NULL) {
        return NULL;
    }

    // Allocate memory for Buck handle
    Buck_HandleTypeDef* handle = malloc(sizeof(Buck_HandleTypeDef));
    if (handle == NULL) {
        return NULL;
    }

    // Load initialization parameters
    (*handle) = (Buck_HandleTypeDef){0}; 
    handle->Init = *init;
    handle->TargetOutputVoltage = 0.0f;
    handle->MaxInductorCurrent = 0.0f;
    handle->Duty = 0.0f;
    handle->State = BUCK_DISABLED;
    handle->FaultCode = BUCK_NOERROR;

    // Call user-defined initialization function
    init->InitFunc();

    //PID initialization
    handle->OutPutVoltagePID = VOLOOP_PID_Init(init->OutPutVoltagePIDInit);
    if (handle->OutPutVoltagePID == NULL) {
        VOLOOP_Buck_DeInit(handle);
        return NULL;
    }
    handle->InductorCurrentPID = VOLOOP_PID_Init(init->InductorCurrentPIDInit);
    if (handle->InductorCurrentPID == NULL) {
        VOLOOP_Buck_DeInit(handle);
        return NULL;
    }

    return handle;
}


void VOLOOP_Buck_DeInit(Buck_HandleTypeDef* handle) {
    // Verify input parameter
    if (handle == NULL) {
        return;
    }

    // Stop the buck converter
    handle->Init.Stop();

    // Call user-defined deinitialization function
    handle->Init.DeInitFunc();

    // Deinitialize PID controllers
    PID_DeInit(handle->OutPutVoltagePID);
    PID_DeInit(handle->InductorCurrentPID);

    // Free Buck handle memory
    free(handle);
}


void VOLOOP_Buck_Start(Buck_HandleTypeDef* handle) {
    if (handle == NULL) {
        return;
    }

    if (handle->State == BUCK_ERROR
        || handle->State == BUCK_CVMODE
        || handle->State == BUCK_CCMODE) {
        return; 
    }
    
    VOLOOP_PID_Reset(handle->OutPutVoltagePID);
    VOLOOP_PID_Reset(handle->InductorCurrentPID);
    handle->State = BUCK_CVMODE; // Default to CV mode when starting
    handle->Init.Start();
}


void VOLOOP_Buck_Stop(Buck_HandleTypeDef* handle) {
    if (handle == NULL) {
        return;
    }
    handle->Init.Stop();
    handle->State = BUCK_DISABLED;
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


void VOLOOP_Buck_ClearFaultCode(Buck_HandleTypeDef* handle) {
    if (handle == NULL) {
        return;
    }
    if (handle->State != BUCK_ERROR
        && handle->State != BUCK_DISABLED) {
        return;
    }
    handle->FaultCode = BUCK_NOERROR;
    handle->State = BUCK_DISABLED;
}


void VOLOOP_Buck_SetValue(Buck_HandleTypeDef* handle, float Voltage, float Current) {
    if (handle == NULL) {
        return;
    }
    handle->TargetOutputVoltage = Voltage;
    handle->MaxInductorCurrent = Current;
}


float VOLOOP_Buck_GetDuty(Buck_HandleTypeDef* handle) {
    if (handle == NULL) {
        return 0.0f;
    }
    return handle->Duty;
}


void VOLOOP_Buck_Sync(Buck_HandleTypeDef* handle) {
    // Verify input parameter
    if (handle == NULL) {
        return;
    }

    if (handle->State == BUCK_ERROR) {
        return; 
    } else if (handle->State == BUCK_DISABLED) {
        return; 
    }

    //Get necessary parameters
    float presentVoltage = handle->Init.GetOutputVoltage();
    float presentCurrent = handle->Init.GetInductorCurrent();

    //Protection
    if ( presentCurrent > BUCK_OCTHRESHOLD){
        Buck_Stop(handle);
        handle->State = BUCK_ERROR;
        handle->FaultCode = BUCK_OCP;
        return;
    }
    if  (presentVoltage > BUCK_OVTHRESHOLD){
        Buck_Stop(handle);
        handle->State = BUCK_ERROR;
        handle->FaultCode = BUCK_OVP;
        return;
    }

    //Compute target inductor current based on output voltage error, with anti-windup
    float targetInductorCurrent = VOLOOP_PID_ComputeConditional(handle->OutPutVoltagePID, handle->TargetOutputVoltage, presentVoltage, 0.0f, handle->MaxInductorCurrent);
    
    if (VOLOOP_PID_GetState(handle->OutPutVoltagePID) == PID_UpperSaturated){
        handle->State = BUCK_CCMODE;
    } else {
        handle->State = BUCK_CVMODE;
    }
    
    float duty = VOLOOP_PID_ComputeConditional(handle->InductorCurrentPID, targetInductorCurrent, presentCurrent, BUCK_MIN_DUTY, BUCK_MAX_DUTY);

    // Set duty cycle
    handle->Duty = duty;
    handle->Init.SetDuty(duty);

}


