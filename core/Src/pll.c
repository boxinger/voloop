#include "pll.h"
#include <stdlib.h>

struct PLL_HandleTypeDef {
    PLL_InitTypeDef Init;
    PID_HandleTypeDef* LPF;
    PLL_StateTypeDef State;
    float InputValue;
    float Phase;
    float Frequency;
    PLL_LockStateTypeDef LockState;
};

PLL_HandleTypeDef* PLL_Init(PLL_InitTypeDef* init) {
    // Verify input parameters
    if (init == NULL 
        || init->InitFunc == NULL
        || init->DeInitFunc == NULL
        || init->GetInputValue == NULL) {
        return NULL;
    }

    // Allocate memory for PLL handle
    PLL_HandleTypeDef* handle = malloc(sizeof(PLL_HandleTypeDef));
    if (handle == NULL) {
        return NULL;
    }

    // Load initialization parameters
    (*handle) = (PLL_HandleTypeDef){0}; 
    handle->Init = *init;
    handle->InputValue = 0.0f;
    handle->Phase = 0.0f;
    handle->Frequency = 0.0f;
    handle->LockState = PLL_UNLOCKED;

    // Call user-defined initialization function
    init->InitFunc();

    // LPF initialization
    if (VOLOOP_PID_Init(&(handle->LPF), init->LPFInit) != VOLOOP_OK) {
        PLL_DeInit(handle);
        return NULL;
    }

    return handle;
}


void PLL_DeInit(PLL_HandleTypeDef* handle) {
    // Verify input parameter
    if (handle == NULL) {
        return;
    }

    // Call user-defined deinitialization function
    handle->Init.DeInitFunc();

    // Deinitialize LPF
    VOLOOP_PID_DeInit(&(handle->LPF));

    // Free PLL handle memory
    free(handle);
}


void PLL_Start(PLL_HandleTypeDef* handle) {
    if (handle == NULL) {
        return;
    }
    if (handle->State == PLL_ERROR) {
        return;
    }

    handle->State = PLL_RUNNING;
}


void PLL_Stop(PLL_HandleTypeDef* handle) {
    if (handle == NULL) {
        return;
    }
    if (handle->State == PLL_ERROR) {
        return;
    }

    handle->State = PLL_STOPPED;
}


PLL_StateTypeDef PLL_GetState(PLL_HandleTypeDef* handle) {
    if (handle == NULL) {
        return PLL_ERROR;
    }
    return handle->State;
}


PLL_LockStateTypeDef PLL_IsLocked(PLL_HandleTypeDef* handle) {
    if (handle == NULL) {
        return PLL_UNLOCKED;
    }
    return handle->LockState;
}


float PLL_GetPhase(PLL_HandleTypeDef* handle) {
    if (handle == NULL) {
        return 0.0f;
    }
    return handle->Phase;
}


float PLL_GetFrequency(PLL_HandleTypeDef* handle) {
    if (handle == NULL) {
        return 0.0f;
    }
    return handle->Frequency;
}


void PLL_Sync(PLL_HandleTypeDef* handle) {
    if (handle == NULL) {
        return;
    }

    
}
