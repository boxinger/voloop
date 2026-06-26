#include "voloop_pll.h"
#include "voloop_def.h"

#include <math.h>

#define PLL_LOCK_PHASE_ERR_THRESHOLD   1.0f  // Phase error threshold to enter LOCKED
#define PLL_UNLOCK_PHASE_ERR_THRESHOLD 1.5f  // Phase error threshold to leave LOCKED (hysteresis)
#define PLL_LOCK_FREQ_ERR_THRESHOLD    3.0f  // Frequency correction threshold to enter LOCKED
#define PLL_UNLOCK_FREQ_ERR_THRESHOLD  5.0f  // Frequency correction threshold to leave LOCKED (hysteresis)
#define PLL_FREQ_MIN                   40.0f // Allowed lock frequency window lower bound (Hz)
#define PLL_FREQ_MAX                   70.0f // Allowed lock frequency window upper bound (Hz)
#define PLL_SIGNAL_MIN_PEAK            0.15f // Minimum input peak to consider signal valid
#define PLL_PEAK_EPSILON               1e-6f // Guard threshold to avoid division by zero on peak
#define PLL_LOCK_DEBOUNCE_COUNT        50    // Consecutive lock samples required to assert LOCKED
#define PLL_UNLOCK_DEBOUNCE_COUNT      20    // Consecutive unlock samples required to assert UNLOCKED
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
    handle->LockCounter = 0;
    handle->UnlockCounter = 0;
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
    handle->LockCounter = 0;
    handle->UnlockCounter = 0;
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
    handle->LockCounter = 0;
    handle->UnlockCounter = 0;
    handle->LockState = PLL_UNLOCKED;
    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_PLL_Reset(PLL_HandleTypeDef* handle) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    // ERROR and uninitialized (RESET) states are not allowed to reset.
    if (handle->State == PLL_ERROR || handle->State == PLL_RESET) {
        return VOLOOP_INVALID_STATE;
    }

    // If running, stop the NCO first (reuse its cleanup path).
    if (handle->State == PLL_RUNNING) {
        VOLOOP_StatusTypeDef status = VOLOOP_NCO_Stop(&(handle->NCO));
        if (status != VOLOOP_OK) {
            handle->State = PLL_ERROR;
            handle->LockState = PLL_UNLOCKED;
            return status;
        }
    }

    // Reset loop filter and oscillator while keeping the init configuration
    // (Kp/Ki, alpha, NominalFrequency, etc.).
    VOLOOP_PID_Reset(&(handle->LoopFilter));
    VOLOOP_NCO_SetFrequency(&(handle->NCO), handle->NominalFrequency);
    VOLOOP_NCO_SetRad(&(handle->NCO), 0.0f);

    // Reset PLL-level state and amplitude estimation.
    handle->PhaseQ31 = 0;
    handle->Frequency = handle->NominalFrequency;
    handle->dcValue = 0.0f;
    handle->squareAvg = 1.0f;
    handle->peakValue = 1.0f;
    handle->peakValueInv = 1.0f;
    handle->InvPeakUpdateCounter = PLL_INV_PEAK_UPDATE_PERIOD;
    handle->LockCounter = 0;
    handle->UnlockCounter = 0;
    handle->LockState = PLL_UNLOCKED;

    // After reset the PLL returns to STOPPED; user must Start again to run.
    handle->State = PLL_STOPPED;
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

    if (handle->State == PLL_STOPPED) {
        return VOLOOP_OK;
    } else if (handle->State != PLL_RUNNING) {
        return VOLOOP_INVALID_STATE;
    }

    handle->dcValue += handle->dcAlpha * (input->InputVoltage - handle->dcValue);
    float acValue = input->InputVoltage - handle->dcValue;
    handle->squareAvg += handle->squareAlpha * (acValue * acValue - handle->squareAvg);
    handle->InvPeakUpdateCounter++;
    if (handle->InvPeakUpdateCounter >= PLL_INV_PEAK_UPDATE_PERIOD) {
        handle->InvPeakUpdateCounter = 0;
        handle->peakValue = sqrtf(handle->squareAvg * 2.0f);
        if (handle->peakValue < PLL_PEAK_EPSILON) {
            // Peak too small (or signal absent): avoid division by zero / blowup.
            // Setting the inverse gain to 0 makes the normalized input 0, so the
            // phase error is 0 and the loop coasts at the nominal frequency until
            // a valid signal returns (also reported as PLL_NO_SIGNAL by lock detect).
            handle->peakValueInv = 0.0f;
        } else {
            handle->peakValueInv = 1.0f / handle->peakValue;
        }
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

    // 4) Lock detection: signal validity -> frequency window -> debounce + hysteresis
    if (handle->peakValue <= PLL_SIGNAL_MIN_PEAK) {
        // Input signal is absent or too weak: cannot judge lock, avoid false LOCKED.
        // Loop keeps running so it can re-acquire quickly once the signal returns.
        handle->LockCounter = 0;
        handle->UnlockCounter = 0;
        handle->LockState = PLL_NO_SIGNAL;
        return VOLOOP_OK;
    }

    uint8_t freqInWindow =
        (handle->Frequency >= PLL_FREQ_MIN) && (handle->Frequency <= PLL_FREQ_MAX);
    float absPhaseError = fabsf(phaseError);
    float absFreqCorrection = fabsf(frequencyCorrection);

    uint8_t lockCondition = freqInWindow &&
                            (absPhaseError < PLL_LOCK_PHASE_ERR_THRESHOLD) &&
                            (absFreqCorrection < PLL_LOCK_FREQ_ERR_THRESHOLD);
    uint8_t unlockCondition = (!freqInWindow) ||
                              (absPhaseError >= PLL_UNLOCK_PHASE_ERR_THRESHOLD) ||
                              (absFreqCorrection >= PLL_UNLOCK_FREQ_ERR_THRESHOLD);

    if (lockCondition) {
        handle->UnlockCounter = 0;
        if (handle->LockCounter < PLL_LOCK_DEBOUNCE_COUNT) {
            handle->LockCounter++;
        }
        if (handle->LockCounter >= PLL_LOCK_DEBOUNCE_COUNT) {
            handle->LockState = PLL_LOCKED;
        }
    } else if (unlockCondition) {
        handle->LockCounter = 0;
        if (handle->UnlockCounter < PLL_UNLOCK_DEBOUNCE_COUNT) {
            handle->UnlockCounter++;
        }
        if (handle->UnlockCounter >= PLL_UNLOCK_DEBOUNCE_COUNT) {
            handle->LockState = PLL_UNLOCKED;
        }
    }
    // else: inside the hysteresis dead-band, hold current LockState and counters.

    return VOLOOP_OK;
}
