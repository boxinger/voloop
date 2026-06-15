#include "voloop_pll.h"
#include "voloop_def.h"

#include <math.h>

#define PLL_LOCK_PHASE_ERR_THRESHOLD 1.0f
#define PLL_LOCK_FREQ_ERR_THRESHOLD 3.0f
#define PLL_ALPHA_MIN  0.000001f
#define PLL_ALPHA_MAX  0.2f
#define PLL_DC_ALPHA_CYCLES 100.0f
#define PLL_SQUARE_ALPHA_CYCLES 5.0f
#define PLL_INV_PEAK_UPDATE_PERIOD 20

static float VOLOOP_PLL_CalcAlphaByCycles(
    float triggerFrequency,
    float nominalFrequency,
    float trackCycles
) {
    if (triggerFrequency <= 0.0f ||
        nominalFrequency <= 0.0f ||
        trackCycles <= 0.0f) {
        return 0.0f;
    }

    float alpha = nominalFrequency / (triggerFrequency * trackCycles);

    return VOLOOP_DEF_ClampFloat(alpha, PLL_ALPHA_MIN, PLL_ALPHA_MAX);
}

VOLOOP_StatusTypeDef VOLOOP_PLL_Init(PLL_HandleTypeDef* handle, const PLL_InitTypeDef* init) {
    // Verify input parameters
    if (handle == NULL || init == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    if (init->LoopFilterInit == NULL || init->NCOInit == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    if (init->triggerFrequency <= 0.0f) {
        return VOLOOP_INVALID_PARAM;
    }

    // Load initialization parameters
    (*handle) = (PLL_HandleTypeDef){ 0 };
    handle->NominalFrequency = init->NCOInit->initialFrequency;
    handle->State = PLL_STOPPED;
    handle->PhaseQ31 = 0;
    handle->Frequency = init->NCOInit->initialFrequency;
    handle->LockState = PLL_UNLOCKED;
    handle->LoopFilter = (PID_HandleTypeDef){ 0 };
    handle->NCO = (NCO_HandleTypeDef){ 0 };
    handle->triggerFrequency = init->triggerFrequency;
    handle->dcAlpha = VOLOOP_PLL_CalcAlphaByCycles(init->triggerFrequency, init->NCOInit->initialFrequency, PLL_DC_ALPHA_CYCLES);
    handle->squareAlpha = VOLOOP_PLL_CalcAlphaByCycles(init->triggerFrequency, init->NCOInit->initialFrequency, PLL_SQUARE_ALPHA_CYCLES);
    handle->dcValue = 0.0f;
    handle->squareAvg = 1.0f;
    handle->peakValue = 1.0f;
    handle->peakValueInv = 1.0f;
    handle->InvPeakUpdateCounter = PLL_INV_PEAK_UPDATE_PERIOD;

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
    if (handle->State == PLL_RUNNING) {
        return VOLOOP_OK;
    } else if (handle->State != PLL_STOPPED) {
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
    if (handle->State == PLL_STOPPED) {
        return VOLOOP_OK;
    } else if (handle->State != PLL_RUNNING) {
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

PLL_StateTypeDef VOLOOP_PLL_GetState(const PLL_HandleTypeDef* handle) {
    if (handle == NULL) {
        return PLL_ERROR;
    }
    return handle->State;
}

PLL_LockStateTypeDef VOLOOP_PLL_IsLocked(const PLL_HandleTypeDef* handle) {
    if (handle == NULL) {
        return PLL_UNLOCKED;
    }
    return handle->LockState;
}

int32_t VOLOOP_PLL_GetPhaseQ31(const PLL_HandleTypeDef* handle) {
    if (handle == NULL) {
        return 0;
    }
    return handle->PhaseQ31;
}

float VOLOOP_PLL_GetRad(const PLL_HandleTypeDef* handle) {
    if (handle == NULL) {
        return 0.0f;
    }
    return VOLOOP_DEF_Q31ToRad(handle->PhaseQ31);
}

float VOLOOP_PLL_GetFrequency(const PLL_HandleTypeDef* handle) {
    if (handle == NULL) {
        return 0.0f;
    }
    return handle->Frequency;
}

VOLOOP_StatusTypeDef VOLOOP_PLL_Sync(PLL_HandleTypeDef* handle, const PLL_InputTypeDef* input) {
    if (handle == NULL || input == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    if (handle->State != PLL_RUNNING) {
        return VOLOOP_INVALID_STATE;
    }

    handle->dcValue += handle->dcAlpha * (input->InputVoltage - handle->dcValue);
    float acValue = input->InputVoltage - handle->dcValue;
    handle->squareAvg += handle->squareAlpha * (acValue * acValue - handle->squareAvg);
    handle->InvPeakUpdateCounter++;
    if (handle->InvPeakUpdateCounter >= PLL_INV_PEAK_UPDATE_PERIOD) {
        handle->InvPeakUpdateCounter = 0;
        handle->peakValue = sqrtf(handle->squareAvg * 2.0f);
        if (handle->peakValue < 0.1f) {
            handle->peakValue = 0.1f;
        }
        handle->peakValueInv = 1.0f / handle->peakValue;
    }

    // 1) Phase detector: input(sin) * NCO(cos)
    float ncoCos = VOLOOP_NCO_GetCosine(&(handle->NCO));
    float inputNormalized = acValue * handle->peakValueInv;
    float phaseError = inputNormalized * ncoCos;

    // 2) Loop filter: PI output as frequency correction
    float frequencyCorrection = VOLOOP_PID_Compute(&(handle->LoopFilter), 0.0f, -phaseError);

    // 3) Update NCO frequency and phase
    // float nextFrequency = VOLOOP_NCO_GetFrequency(handle->NCO) + frequencyCorrection;
    float nextFrequency = handle->NominalFrequency + frequencyCorrection;
    VOLOOP_StatusTypeDef status = VOLOOP_NCO_SetFrequency(&(handle->NCO), nextFrequency);
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
