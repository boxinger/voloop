// Phase-Locked Loop
#ifndef VOLOOP_PLL_H
#define VOLOOP_PLL_H

#include "voloop_pid.h"
#include "voloop_nco.h"
#include "voloop_def.h"

typedef struct {
    float InputVoltage;
} PLL_InputTypeDef;

typedef struct {
    const PID_InitTypeDef* LoopFilterInit;
    const NCO_InitTypeDef* NCOInit;
    const float triggerFrequency;
    // const VOLOOP_StatusTypeDef (*GetInputParam)(PLL_InputTypeDef* input);
} PLL_InitTypeDef;

typedef enum {
    PLL_RESET = 0U,
    PLL_ERROR,
    PLL_STOPPED,
    PLL_RUNNING
} PLL_StateTypeDef;

typedef enum {
    PLL_UNLOCKED = 0U,
    PLL_LOCKED,
    PLL_NO_SIGNAL
} PLL_LockStateTypeDef;

typedef struct {
    PID_HandleTypeDef LoopFilter;
    NCO_HandleTypeDef NCO;
    float NominalFrequency;
    float triggerFrequency;

    // Normalize
    float dcAlpha;
    float squareAlpha;
    float dcValue;
    float squareAvg;
    float peakValue;
    float peakValueInv;
    uint8_t InvPeakUpdateCounter;

    int32_t PhaseQ31;
    float Frequency;
    uint16_t LockCounter;   // Consecutive samples meeting the lock criteria
    uint16_t UnlockCounter; // Consecutive samples failing the lock criteria
    PLL_LockStateTypeDef LockState;
    PLL_StateTypeDef State;
} PLL_HandleTypeDef;

VOLOOP_StatusTypeDef VOLOOP_PLL_Init(PLL_HandleTypeDef* handle, const PLL_InitTypeDef* init);
VOLOOP_StatusTypeDef VOLOOP_PLL_DeInit(PLL_HandleTypeDef* handle);
VOLOOP_StatusTypeDef VOLOOP_PLL_Start(PLL_HandleTypeDef* handle);
VOLOOP_StatusTypeDef VOLOOP_PLL_Stop(PLL_HandleTypeDef* handle);
VOLOOP_StatusTypeDef VOLOOP_PLL_Reset(PLL_HandleTypeDef* handle);
PLL_StateTypeDef VOLOOP_PLL_GetState(const PLL_HandleTypeDef* handle);

PLL_LockStateTypeDef VOLOOP_PLL_IsLocked(const PLL_HandleTypeDef* handle);
int32_t VOLOOP_PLL_GetPhaseQ31(const PLL_HandleTypeDef* handle); // range: -2^31 to 2^31-1
float VOLOOP_PLL_GetRad(const PLL_HandleTypeDef* handle);        // range: [-pi, pi)
float VOLOOP_PLL_GetFrequency(const PLL_HandleTypeDef* handle);

VOLOOP_StatusTypeDef VOLOOP_PLL_Sync(PLL_HandleTypeDef* handle, const PLL_InputTypeDef* input);

#endif
