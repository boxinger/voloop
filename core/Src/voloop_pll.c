#include "voloop_pll.h"
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

VOLOOP_StatusTypeDef VOLOOP_PLL_Init(PLL_HandleTypeDef** handleOut, const PLL_InitTypeDef* init) {
    // Verify input parameters
    if (handleOut == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    if (*handleOut != NULL
        || init == NULL
        || init->InitFunc == NULL
        || init->DeInitFunc == NULL
        || init->GetInputValue == NULL
        || init->LPFInit == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    // Allocate memory for PLL handle
    PLL_HandleTypeDef* handle = malloc(sizeof(PLL_HandleTypeDef));
    if (handle == NULL) {
        return VOLOOP_BAD_ALLOCATE;
    }

    // Load initialization parameters
    (*handle) = (PLL_HandleTypeDef){0};
    handle->Init = *init;
    handle->State = PLL_STOPPED;
    handle->InputValue = 0.0f;
    handle->Phase = 0.0f;
    handle->Frequency = 0.0f;
    handle->LockState = PLL_UNLOCKED;

    // Call user-defined initialization function
    init->InitFunc();

    // LPF initialization
    if (VOLOOP_PID_Init(&(handle->LPF), init->LPFInit) != VOLOOP_OK) {
        init->DeInitFunc();
        free(handle);
        return VOLOOP_ERROR;
    }

    *handleOut = handle;
    return VOLOOP_OK;
}


VOLOOP_StatusTypeDef VOLOOP_PLL_DeInit(PLL_HandleTypeDef** handleOut) {
    // Verify input parameter
    if (handleOut == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    if (*handleOut == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    PLL_HandleTypeDef* handle = *handleOut;

    // Call user-defined deinitialization function
    handle->Init.DeInitFunc();

    // Deinitialize LPF
    VOLOOP_PID_DeInit(&(handle->LPF));

    // Free PLL handle memory
    free(handle);
    *handleOut = NULL;
    return VOLOOP_OK;
}


VOLOOP_StatusTypeDef VOLOOP_PLL_Start(PLL_HandleTypeDef* handle) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    if (handle->State == PLL_ERROR) {
        return VOLOOP_INVALID_STATE;
    }

    handle->State = PLL_RUNNING;
    return VOLOOP_OK;
}


VOLOOP_StatusTypeDef VOLOOP_PLL_Stop(PLL_HandleTypeDef* handle) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    if (handle->State == PLL_ERROR) {
        return VOLOOP_INVALID_STATE;
    }

    handle->State = PLL_STOPPED;
    return VOLOOP_OK;
}


PLL_StateTypeDef VOLOOP_PLL_GetState(PLL_HandleTypeDef* handle) {
    if (handle == NULL) {
        return PLL_ERROR;
    }
    return handle->State;
}


PLL_LockStateTypeDef VOLOOP_PLL_IsLocked(PLL_HandleTypeDef* handle) {
    if (handle == NULL) {
        return PLL_UNLOCKED;
    }
    return handle->LockState;
}


float VOLOOP_PLL_GetPhase(PLL_HandleTypeDef* handle) {
    if (handle == NULL) {
        return 0.0f;
    }
    return handle->Phase;
}


float VOLOOP_PLL_GetFrequency(PLL_HandleTypeDef* handle) {
    if (handle == NULL) {
        return 0.0f;
    }
    return handle->Frequency;
}


VOLOOP_StatusTypeDef VOLOOP_PLL_Sync(PLL_HandleTypeDef* handle) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    if (handle->State == PLL_ERROR || handle->State == PLL_STOPPED) {
        return VOLOOP_INVALID_STATE;
    }

    return VOLOOP_OK;
}
