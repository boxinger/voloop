// Numerically Controlled Oscillator
#ifndef VOLOOP_NCO_H
#define VOLOOP_NCO_H

#include <stdint.h>

typedef struct {
    uint32_t triggerFrequency;
    float initialFrequency;
    float initialPhase;
} NCO_InitTypeDef;

typedef enum {
    NCO_ERROR = 0U,
    NCO_STOPPED,
    NCO_RUNNING
} NCO_StateTypeDef;

typedef struct NCO_HandleTypeDef NCO_HandleTypeDef;

#define NCO_Pi 3.14159265358979323846f
#define NCO_TwoPi (2.0f * NCO_Pi)

NCO_HandleTypeDef* VOLOOP_NCO_Init(NCO_InitTypeDef* init);
void VOLOOP_NCO_DeInit(NCO_HandleTypeDef* handle);

void VOLOOP_NCO_Start(NCO_HandleTypeDef* handle);
void VOLOOP_NCO_Stop(NCO_HandleTypeDef* handle);
NCO_StateTypeDef VOLOOP_NCO_GetState(NCO_HandleTypeDef* handle);

// phase range: [-pi, pi), matching STM32 CORDIC Q1.31 phase format
void VOLOOP_NCO_SetFrequency(NCO_HandleTypeDef* handle, float frequency);
float VOLOOP_NCO_GetFrequency(NCO_HandleTypeDef* handle);
void VOLOOP_NCO_SetPhase(NCO_HandleTypeDef* handle, float phase);
float VOLOOP_NCO_GetPhase(NCO_HandleTypeDef* handle);
int32_t VOLOOP_NCO_GetPhaseQ31(NCO_HandleTypeDef* handle);
const int32_t* VOLOOP_NCO_GetPhaseQ31Address(NCO_HandleTypeDef* handle);
float VOLOOP_NCO_GetSine(NCO_HandleTypeDef* handle);
float VOLOOP_NCO_GetCosine(NCO_HandleTypeDef* handle);

void VOLOOP_NCO_Sync(NCO_HandleTypeDef* handle);

#endif /* __NCO_H */
