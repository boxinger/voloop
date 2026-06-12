#include "voloop_pll.h"
#include "voloop_def.h"

#define PLL_LOCK_PHASE_ERR_THRESHOLD 1.0f
#define PLL_LOCK_FREQ_ERR_THRESHOLD 3.0f

VOLOOP_StatusTypeDef VOLOOP_PLL_Init(PLL_HandleTypeDef* handle, const PLL_InitTypeDef* init) {
    // Verify input parameters
    if (handle == NULL || init == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    if (init->InitFunc == NULL || init->DeInitFunc == NULL || init->GetInputValue == NULL ||
        init->LoopFilterInit == NULL || init->NCOInit == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    // Load initialization parameters
    (*handle) = (PLL_HandleTypeDef){ 0 };
    handle->Init = *init;
    handle->State = PLL_STOPPED;
    handle->InputValue = 0.0f;
    handle->PhaseQ31 = 0;
    handle->Frequency = 0.0f;
    handle->LockState = PLL_UNLOCKED;
    handle->LoopFilter = (PID_HandleTypeDef){ 0 };
    handle->NCO = (NCO_HandleTypeDef){ 0 };

    // Call user-defined initialization function
    init->InitFunc();

    // LoopFilter and NCO initialization
    VOLOOP_StatusTypeDef status;
    status = VOLOOP_PID_Init(&(handle->LoopFilter), init->LoopFilterInit);
    if (status != VOLOOP_OK) {
        VOLOOP_PLL_DeInit(handle);
        return status;
    }

    status = VOLOOP_NCO_Init(&(handle->NCO), init->NCOInit);
    if (status != VOLOOP_OK) {
        VOLOOP_PLL_DeInit(handle);
        return status;
    }

    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_PLL_DeInit(PLL_HandleTypeDef* handle) {
    // Verify input parameter
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    // Call user-defined deinitialization function
    handle->Init.DeInitFunc();

    // Deinitialize LoopFilter
    VOLOOP_PID_DeInit(&(handle->LoopFilter));
    VOLOOP_NCO_DeInit(&(handle->NCO));

    *handle = (PLL_HandleTypeDef){ 0 };
    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_PLL_Start(PLL_HandleTypeDef* handle) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    if (handle->State == PLL_ERROR) {
        return VOLOOP_INVALID_STATE;
    }

    VOLOOP_StatusTypeDef status = VOLOOP_NCO_Start(&(handle->NCO));
    if (status != VOLOOP_OK) {
        handle->State = PLL_ERROR;
        handle->LockState = PLL_UNLOCKED;
        return status;
    }

    VOLOOP_PID_Reset(&(handle->LoopFilter));
    handle->LockState = PLL_UNLOCKED;

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

    VOLOOP_StatusTypeDef status = VOLOOP_NCO_Stop(&(handle->NCO));
    if (status != VOLOOP_OK) {
        handle->State = PLL_ERROR;
        handle->LockState = PLL_UNLOCKED;
        return status;
    }

    handle->State = PLL_STOPPED;
    handle->LockState = PLL_UNLOCKED;
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

int32_t VOLOOP_PLL_GetPhaseQ31(PLL_HandleTypeDef* handle) {
    if (handle == NULL) {
        return 0;
    }
    return handle->PhaseQ31;
}

float VOLOOP_PLL_GetRad(PLL_HandleTypeDef* handle) {
    if (handle == NULL) {
        return 0.0f;
    }
    return VOLOOP_DEF_Q31ToRad(handle->PhaseQ31);
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

    // 1) Phase detector: input(sin) * NCO(cos)
    float inputValue = handle->Init.GetInputValue();
    float ncoCos = VOLOOP_NCO_GetCosine(&(handle->NCO));
    float phaseError = inputValue * ncoCos;

    // 2) Loop filter: PI output as frequency correction
    float frequencyCorrection = VOLOOP_PID_Compute(&(handle->LoopFilter), 0.0f, -phaseError);

    // 3) Update NCO frequency and phase
    // float nextFrequency = VOLOOP_NCO_GetFrequency(handle->NCO) + frequencyCorrection;
    float nextFrequency = handle->Init.NCOInit->initialFrequency + frequencyCorrection;
    volatile VOLOOP_StatusTypeDef status = VOLOOP_NCO_SetFrequency(&(handle->NCO), nextFrequency);
    if (status != VOLOOP_OK) {
        handle->State = PLL_ERROR;
        handle->LockState = PLL_UNLOCKED;
        return status;
    }

    status = VOLOOP_NCO_Sync(&(handle->NCO));
    if (status != VOLOOP_OK) {
        handle->State = PLL_ERROR;
        handle->LockState = PLL_UNLOCKED;
        return status;
    }

    handle->InputValue = inputValue;
    handle->PhaseQ31 = VOLOOP_NCO_GetPhaseQ31(&(handle->NCO));
    handle->Frequency = VOLOOP_NCO_GetFrequency(&(handle->NCO));

    // 4) Simple lock detection
    if ((fabsf(phaseError) < PLL_LOCK_PHASE_ERR_THRESHOLD) &&
        (fabsf(frequencyCorrection) < PLL_LOCK_FREQ_ERR_THRESHOLD)) {
        handle->LockState = PLL_LOCKED;
    } else {
        handle->LockState = PLL_UNLOCKED;
    }

    return VOLOOP_OK;
}
