/*
    Discrete form used by runtime:
        H(z) = (b0 + b1 * z^-1) / (1 + a1 * z^-1)

    Continuous form accepted by config:
        H(s) = K * (b0 * s + b1) / (a0 * s + a1)
*/
#ifndef VOLOOP_FOF_H
#define VOLOOP_FOF_H
#include "voloop_def.h"

#include <stdint.h>

typedef enum {
    FOF_Discrete = 0U,
    FOF_Continue,
    FOF_LowPass,
    FOF_HighPass,
    FOF_LeadLag,
} FOF_InitModeTypeDef;

typedef struct {
    float b0;
    float b1;
    float a1;
} FOF_InitDiscreteTypeDef;

typedef struct {
    float K;
    float b0;
    float b1;
    float a0;
    float a1;
    float triggerFrequency;
} FOF_InitContinueTypeDef;

typedef struct {
    float cutoffFrequency; // in Hz
    float triggerFrequency;
} FOF_InitLowPassTypeDef;

typedef struct {
    float cutoffFrequency; // in Hz
    float triggerFrequency;
} FOF_InitHighPassTypeDef;

typedef struct {
    float zero;
    float pole;
    float gain;
    float triggerFrequency;
} FOF_InitLeadLagTypeDef;

typedef struct {
    FOF_InitModeTypeDef mode;
    union {
        FOF_InitDiscreteTypeDef Discrete;
        FOF_InitContinueTypeDef Continue;
        FOF_InitLowPassTypeDef LowPass;
        FOF_InitHighPassTypeDef HighPass;
        FOF_InitLeadLagTypeDef LeadLag;
    } init;
} FOF_InitTypeDef;

/*
Discrete Transfer function:
    H(z) = (b0 + b1 * z^-1 + ... + bN * z^-N) / (1 + a1 * z^-1 + ... + aM * z^-M)
Continue Transfer function:
    H(s) = K* (b0 * s^N + b1 * s^(N-1) + ... + bN) / (a0 * s^M + a1 * s^(M-1) + ... + aM)
*/
typedef struct {
    // Initialization parameters
    float b0;
    float b1;
    float a1;

    // Previous input and output
    float y1;
    float x1;
} FOF_HandleTypeDef;

VOLOOP_StatusTypeDef VOLOOP_FOF_Init(FOF_HandleTypeDef* handle, const FOF_InitTypeDef* init);
VOLOOP_StatusTypeDef VOLOOP_FOF_DeInit(FOF_HandleTypeDef* handle);

VOLOOP_StatusTypeDef VOLOOP_FOF_Reconfig(FOF_HandleTypeDef* handle, const FOF_InitTypeDef* init);
VOLOOP_StatusTypeDef VOLOOP_FOF_Reset(FOF_HandleTypeDef* handle);
VOLOOP_StatusTypeDef VOLOOP_FOF_ResetWithValue(FOF_HandleTypeDef* handle, float input,
                                               float output);

float VOLOOP_FOF_Compute(FOF_HandleTypeDef* handle, float input);

#endif