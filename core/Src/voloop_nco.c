#include "voloop_nco.h"
#include "voloop_def.h"
#include <math.h>
#include <stdlib.h>

#ifndef NCO_SIN_FUNC
#define NCO_SIN_FUNC(x) sinf(x)
#endif

#ifndef NCO_COS_FUNC
#define NCO_COS_FUNC(x) cosf(x)
#endif

#define NCO_Q31_SCALE 2147483648.0f
#define NCO_U32_SCALE 4294967296.0f
#define NCO_PI_INV (1.0f / VOLOOP_Pi)
#define NCO_Q31_SCALE_INV (1.0f / NCO_Q31_SCALE)

struct NCO_HandleTypeDef {
    NCO_InitTypeDef Init;
    NCO_StateTypeDef State;
    float Frequency;
    float TriggerFrequencyInv;
    int32_t PhaseQ31;
    uint32_t PhaseStepQ31;
};

static int32_t NCO_PhaseToQ31(float phase) {
    return (int32_t)(phase * NCO_Q31_SCALE * NCO_PI_INV);
}

static float NCO_Q31ToPhase(int32_t phaseQ31) {
    return (float)phaseQ31 * VOLOOP_Pi * NCO_Q31_SCALE_INV;
}

VOLOOP_StatusTypeDef VOLOOP_NCO_Init(NCO_HandleTypeDef** handleOut, const NCO_InitTypeDef* init) {
    // Verify input parameters
    if (handleOut == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    if (*handleOut != NULL || init == NULL
        || init->triggerFrequency == 0U 
        || init->initialFrequency <= 0.0f
        || init->initialFrequency >= (float)init->triggerFrequency
        || init->initialPhase < -VOLOOP_Pi
        || init->initialPhase >= VOLOOP_Pi
    ) {
        return VOLOOP_INVALID_PARAM;
    }

    // Allocate memory for NCO handle
    NCO_HandleTypeDef* handle = malloc(sizeof(NCO_HandleTypeDef));
    if (handle == NULL) {
        return VOLOOP_BAD_ALLOCATE;
    }

    // Load initialization parameters
    (*handle) = (NCO_HandleTypeDef){0};
    handle->Init = *init;
    handle->State = NCO_STOPPED;
    handle->Frequency = init->initialFrequency;
    handle->TriggerFrequencyInv = 1.0f / (float)init->triggerFrequency;
    handle->PhaseQ31 = NCO_PhaseToQ31(init->initialPhase);
    handle->PhaseStepQ31 = (uint32_t)(NCO_U32_SCALE * handle->Frequency * handle->TriggerFrequencyInv);
    *handleOut = handle;

    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_NCO_DeInit(NCO_HandleTypeDef** handleOut) {
    // Verify input parameter
    if (handleOut == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    if (*handleOut == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    // Free NCO handle memory
    free(*handleOut);
    *handleOut = NULL;
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

VOLOOP_StatusTypeDef VOLOOP_NCO_SetFrequency(NCO_HandleTypeDef* handle, float frequency) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    if (frequency <= 0.0f || frequency >= (float)handle->Init.triggerFrequency) {
        handle->Frequency = 0.0f;
        handle->PhaseStepQ31 = 0U;
        handle->State = NCO_ERROR;
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

VOLOOP_StatusTypeDef VOLOOP_NCO_SetPhase(NCO_HandleTypeDef* handle, float phase) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    if (phase < -VOLOOP_Pi || phase >= VOLOOP_Pi) {
        handle->State = NCO_ERROR;
        return VOLOOP_INVALID_PARAM;
    }

    handle->PhaseQ31 = NCO_PhaseToQ31(phase);
    return VOLOOP_OK;
}

float VOLOOP_NCO_GetPhase(NCO_HandleTypeDef* handle) {
    if (handle == NULL) {
        return 0.0f;
    }

    return NCO_Q31ToPhase(handle->PhaseQ31);
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

    return NCO_SIN_FUNC(NCO_Q31ToPhase(handle->PhaseQ31));
}

float VOLOOP_NCO_GetCosine(NCO_HandleTypeDef* handle) {
    if (handle == NULL) {
        return 0.0f;
    }

    return NCO_COS_FUNC(NCO_Q31ToPhase(handle->PhaseQ31));
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
