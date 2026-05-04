#ifndef VOLOOP_PLL_H
#define VOLOOP_PLL_H

#include "pid.h"

typedef struct {
    void (*InitFunc)(void);
    void (*DeInitFunc)(void);
    float (*GetInputValue)(void);
    PID_InitTypeDef* LPFInit;
} PLL_InitTypeDef;

typedef enum {
    PLL_ERROR = 0U,
    PLL_DISABLED,
    PLL_RUNNING
} PLL_StateTypeDef;

typedef enum {
    PLL_UNLOCKED = 0U,
    PLL_LOCKED
} PLL_LockStateTypeDef;

typedef struct PLL_HandleTypeDef PLL_HandleTypeDef;

PLL_HandleTypeDef* PLL_Init(PLL_InitTypeDef* init);
void PLL_DeInit(PLL_HandleTypeDef* handle);

PLL_LockStateTypeDef PLL_IsLocked(PLL_HandleTypeDef* handle);
float PLL_GetPhase(PLL_HandleTypeDef* handle);  // range: -2pi to 2pi
float PLL_GetFrequency(PLL_HandleTypeDef* handle);
float PLL_GetSine(PLL_HandleTypeDef* handle);
float PLL_GetCosine(PLL_HandleTypeDef* handle);

void PLL_Sync(PLL_HandleTypeDef* handle);

#endif
