// Numerically Controlled Oscillator
#ifndef VOLOOP_NCO_H
#define VOLOOP_NCO_H

#include "voloop_def.h"
#include <stdint.h>

typedef struct {
    uint32_t triggerFrequency;
    float initialFrequency;
    float initialRad;
} NCO_InitTypeDef;

typedef enum {
    NCO_ERROR = 0U,
    NCO_STOPPED,
    NCO_RUNNING
} NCO_StateTypeDef;

typedef struct {
    NCO_InitTypeDef Init;
    NCO_StateTypeDef State;
    float Frequency;
    float TriggerFrequencyInv;
    int32_t PhaseQ31;
    uint32_t PhaseStepQ31;
} NCO_HandleTypeDef;

VOLOOP_StatusTypeDef VOLOOP_NCO_Init(NCO_HandleTypeDef* handle, const NCO_InitTypeDef* init);
VOLOOP_StatusTypeDef VOLOOP_NCO_DeInit(NCO_HandleTypeDef* handle);

VOLOOP_StatusTypeDef VOLOOP_NCO_Start(NCO_HandleTypeDef* handle);
VOLOOP_StatusTypeDef VOLOOP_NCO_Stop(NCO_HandleTypeDef* handle);
NCO_StateTypeDef VOLOOP_NCO_GetState(NCO_HandleTypeDef* handle);
VOLOOP_StatusTypeDef VOLOOP_NCO_ClearFaultCode(NCO_HandleTypeDef* handle);

// rad range: [-pi, pi), matching STM32 CORDIC Q1.31 phase format
VOLOOP_StatusTypeDef VOLOOP_NCO_SetFrequency(NCO_HandleTypeDef* handle, float frequency);
float VOLOOP_NCO_GetFrequency(NCO_HandleTypeDef* handle);
VOLOOP_StatusTypeDef VOLOOP_NCO_SetRad(NCO_HandleTypeDef* handle, float rad);
float VOLOOP_NCO_GetRad(NCO_HandleTypeDef* handle);
int32_t VOLOOP_NCO_GetPhaseQ31(NCO_HandleTypeDef* handle);
const int32_t* VOLOOP_NCO_GetPhaseQ31Address(NCO_HandleTypeDef* handle);
float VOLOOP_NCO_GetSine(NCO_HandleTypeDef* handle);
float VOLOOP_NCO_GetCosine(NCO_HandleTypeDef* handle);

VOLOOP_StatusTypeDef VOLOOP_NCO_Sync(NCO_HandleTypeDef* handle);

#endif /* VOLOOP_NCO_H */
