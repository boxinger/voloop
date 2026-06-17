/*
    Quasi/Proportional-Resonant (QPR) controller.

    Ideal PR (infinite gain at resonance):
        G(s) = Kp + Kr * s / (s^2 + w0^2)

    Non-ideal / quasi PR (finite gain at resonance, damped):
        G(s) = Kp + Kr * (2 * wc * s) / (s^2 + 2 * wc * s + w0^2)

    where:
        w0 = 2 * pi * resonantFrequency   (resonant angular frequency)
        wc = 2 * pi * cutoffFrequency     (resonant bandwidth, half-power)

    Discrete form used by runtime:
        H(z) = (b0 + b1 * z^-1 + b2 * z^-2) / (1 + a1 * z^-1 + a2 * z^-2)
*/
#ifndef VOLOOP_QPR_H
#define VOLOOP_QPR_H
#include "voloop_def.h"

#include <stdint.h>

typedef enum {
    QPR_Discrete = 0U,
    QPR_Ideal,
    QPR_NonIdeal,
} QPR_InitModeTypeDef;

typedef enum {
    QPR_ERROR = 0U, // Reserved
    QPR_Unsaturated,
    QPR_UpperSaturated,
    QPR_LowerSaturated
} QPR_StateTypeDef;

typedef struct {
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
} QPR_InitDiscreteTypeDef;

typedef struct {
    float Kp;
    float Kr;
    float resonantFrequency; // in Hz
    float triggerFrequency;  // control loop frequency in Hz
} QPR_InitIdealTypeDef;

typedef struct {
    float Kp;
    float Kr;
    float resonantFrequency; // in Hz
    float cutoffFrequency;   // resonant bandwidth in Hz
    float triggerFrequency;  // control loop frequency in Hz
} QPR_InitNonIdealTypeDef;

typedef struct {
    QPR_InitModeTypeDef mode;
    union {
        QPR_InitDiscreteTypeDef Discrete;
        QPR_InitIdealTypeDef Ideal;
        QPR_InitNonIdealTypeDef NonIdeal;
    } init;
} QPR_InitTypeDef;

/*
Discrete Transfer function:
    H(z) = (b0 + b1 * z^-1 + b2 * z^-2) / (1 + a1 * z^-1 + a2 * z^-2)
*/
typedef struct {
    // Initialization parameters
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;

    // Previous inputs and outputs
    float x1;
    float x2;
    float y1;
    float y2;

    QPR_StateTypeDef State;
} QPR_HandleTypeDef;

VOLOOP_StatusTypeDef VOLOOP_QPR_Init(QPR_HandleTypeDef* handle, const QPR_InitTypeDef* init);
VOLOOP_StatusTypeDef VOLOOP_QPR_DeInit(QPR_HandleTypeDef* handle);

VOLOOP_StatusTypeDef VOLOOP_QPR_Reconfig(QPR_HandleTypeDef* handle, const QPR_InitTypeDef* init);
VOLOOP_StatusTypeDef VOLOOP_QPR_Reset(QPR_HandleTypeDef* handle);
VOLOOP_StatusTypeDef VOLOOP_QPR_ResetWithValue(QPR_HandleTypeDef* handle, float x1, float x2,
                                               float y1, float y2);
QPR_StateTypeDef VOLOOP_QPR_GetState(QPR_HandleTypeDef* handle);

float VOLOOP_QPR_Compute(QPR_HandleTypeDef* handle, float input);

// Back-calculation anti-windup
// Kb should be greater than 0
float VOLOOP_QPR_ComputeBackCalculation(QPR_HandleTypeDef* handle, float input, float outputMin,
                                        float outputMax, float Kb);

#endif /* VOLOOP_QPR_H */
