#include "voloop_nco.h"
#include "voloop_def.h"

#define NCO_PHASE_ACCUMULATOR_BITS      32U
#define NCO_U32_SCALE                   ((float)(1ULL << NCO_PHASE_ACCUMULATOR_BITS))

static int32_t NCO_RadToQ31(float rad) {
    return VOLOOP_DEF_RadToQ31(rad);
}

static float NCO_Q31ToRad(int32_t phaseQ31) {
    return VOLOOP_DEF_Q31ToRad(phaseQ31);
}

VOLOOP_StatusTypeDef VOLOOP_NCO_Init(NCO_HandleTypeDef* handle, const NCO_InitTypeDef* init) {
    // Verify input parameters
    if (handle == NULL
        || init == NULL
    ) {
        return VOLOOP_INVALID_PARAM;
    }
    if (init->triggerFrequency == 0U 
        || init->initialFrequency <= 0.0f
        || init->initialFrequency >= (float)init->triggerFrequency
        || init->initialRad < -VOLOOP_Pi
        || init->initialRad >= VOLOOP_Pi
    ) {
        return VOLOOP_INVALID_PARAM;
    }

    // Load initialization parameters
    (*handle) = (NCO_HandleTypeDef){0};
    handle->Init = *init;
    handle->State = NCO_STOPPED;
    handle->Frequency = init->initialFrequency;
    handle->TriggerFrequencyInv = 1.0f / (float)init->triggerFrequency;
    handle->PhaseQ31 = NCO_RadToQ31(init->initialRad);
    handle->PhaseStepQ31 = (uint32_t)(NCO_U32_SCALE * handle->Frequency * handle->TriggerFrequencyInv);

    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_NCO_DeInit(NCO_HandleTypeDef* handle) {
    // Verify input parameter
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    *handle = (NCO_HandleTypeDef){0};
    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_NCO_Start(NCO_HandleTypeDef* handle) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    if (handle->State == NCO_ERROR) {
        return VOLOOP_INVALID_STATE;
    }

    handle->State = NCO_RUNNING;
    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_NCO_Stop(NCO_HandleTypeDef* handle) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    if (handle->State == NCO_ERROR) {
        return VOLOOP_INVALID_STATE;
    }

    handle->State = NCO_STOPPED;
    return VOLOOP_OK;
}

NCO_StateTypeDef VOLOOP_NCO_GetState(NCO_HandleTypeDef* handle) {
    if (handle == NULL) {
        return NCO_ERROR;
    }

    return handle->State;
}

VOLOOP_StatusTypeDef VOLOOP_NCO_ClearFaultCode(NCO_HandleTypeDef* handle) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    if (handle->State != NCO_ERROR) {
        return VOLOOP_INVALID_STATE;
    }

    handle->State = NCO_STOPPED;
    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_NCO_SetFrequency(NCO_HandleTypeDef* handle, float frequency) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    if (frequency <= 0.0f || frequency >= (float)handle->Init.triggerFrequency) {
        return VOLOOP_INVALID_PARAM;
    }

    handle->Frequency = frequency;
    handle->PhaseStepQ31 = (uint32_t)(NCO_U32_SCALE * handle->Frequency * handle->TriggerFrequencyInv);
    return VOLOOP_OK;
}

float VOLOOP_NCO_GetFrequency(NCO_HandleTypeDef* handle) {
    if (handle == NULL) {
        return 0.0f;
    }

    return handle->Frequency;
}

VOLOOP_StatusTypeDef VOLOOP_NCO_SetRad(NCO_HandleTypeDef* handle, float rad) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    if (rad < -VOLOOP_Pi || rad >= VOLOOP_Pi) {
        handle->State = NCO_ERROR;
        return VOLOOP_INVALID_PARAM;
    }

    handle->PhaseQ31 = NCO_RadToQ31(rad);
    return VOLOOP_OK;
}

float VOLOOP_NCO_GetRad(NCO_HandleTypeDef* handle) {
    if (handle == NULL) {
        return 0.0f;
    }

    return NCO_Q31ToRad(handle->PhaseQ31);
}

int32_t VOLOOP_NCO_GetPhaseQ31(NCO_HandleTypeDef* handle) {
    if (handle == NULL) {
        return 0;
    }

    return handle->PhaseQ31;
}

const int32_t* VOLOOP_NCO_GetPhaseQ31Address(NCO_HandleTypeDef* handle) {
    if (handle == NULL) {
        return NULL;
    }

    return &handle->PhaseQ31;
}

float VOLOOP_NCO_GetSine(NCO_HandleTypeDef* handle) {
    if (handle == NULL) {
        return 0.0f;
    }

    return VOLOOP_DEF_SINQ31(handle->PhaseQ31);
}

float VOLOOP_NCO_GetCosine(NCO_HandleTypeDef* handle) {
    if (handle == NULL) {
        return 0.0f;
    }

    return VOLOOP_DEF_COSQ31(handle->PhaseQ31);
}

VOLOOP_StatusTypeDef VOLOOP_NCO_Sync(NCO_HandleTypeDef* handle) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    if (handle->State == NCO_ERROR || handle->State == NCO_STOPPED) {
        return VOLOOP_INVALID_STATE;
    }

    handle->PhaseQ31 = (int32_t)((uint32_t)handle->PhaseQ31 + handle->PhaseStepQ31);
    return VOLOOP_OK;
}
