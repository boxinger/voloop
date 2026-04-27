#ifndef __PID_H
#define __PID_H

#include <stdint.h>

#define PID_Pi 3.14159265358979323846f
#define PID_TwoPi (2.0f * PID_Pi)
#define PID_FourPiSquared (4.0f * PID_Pi * PID_Pi)

typedef struct {
    float KpDiscrete;
    float KiDiscrete;
    float KdDiscrete;
} PID_InitDiscreteTypeDef;


typedef struct {
    float Kp;
    float Ki;
    float Kd;
    uint32_t triggerFrequency; // Control loop frequency in Hz
} PID_InitContinueTypeDef;


// The unit of zero is in Hz, not in rad/s
typedef struct {
    float gain;
    float zero;
    uint32_t triggerFrequency;
} PID_Init1ZeroTypeDef;


typedef struct {
    float gain;
    float zero1;
    float zero2;
    uint32_t triggerFrequency;
} PID_Init2ZeroTypeDef;


typedef enum {
    PID_ERROR = 0U, // Reserved
    PID_UnSaturated,
    PID_UpperSaturated,
    PID_LowerSaturated
} PID_StateTypeDef;


typedef struct PID_HandleTypeDef PID_HandleTypeDef;

typedef enum {
    PID_Discrete = 0U,
    PID_Continue,
    PID_OneZero,
    PID_TwoZero
} PID_InitModeTypeDef;

typedef struct {
    PID_InitModeTypeDef mode;
    union {
        PID_InitDiscreteTypeDef Discrete;
        PID_InitContinueTypeDef Continue;
        PID_Init1ZeroTypeDef OneZero;
        PID_Init2ZeroTypeDef TwoZero;
    } init;
} PID_InitTypeDef;

/*
#define PID_Init(init) \
    _Generic((init), \
        PID_InitDiscreteTypeDef*: PID_InitDiscrete, \
        PID_InitContinueTypeDef*: PID_InitContinue, \
        PID_Init1ZeroTypeDef*: PID_Init1Zero, \
        PID_Init2ZeroTypeDef*: PID_Init2Zero \
    )(init)
*/

PID_HandleTypeDef* PID_Init(const PID_InitTypeDef* init);
PID_HandleTypeDef* PID_InitDiscrete(const PID_InitDiscreteTypeDef* init);
PID_HandleTypeDef* PID_InitContinue(const PID_InitContinueTypeDef* init);
PID_HandleTypeDef* PID_Init1Zero(const PID_Init1ZeroTypeDef* init);
PID_HandleTypeDef* PID_Init2Zero(const PID_Init2ZeroTypeDef* init);
void PID_DeInit(PID_HandleTypeDef* handle);
void PID_Reset(PID_HandleTypeDef* handle);
void PID_SetIntegral(PID_HandleTypeDef* handle, float integral) ;
void PID_SetPreviousError(PID_HandleTypeDef* handle, float previousError);
PID_StateTypeDef PID_GetState(PID_HandleTypeDef* handle);

float PID_Compute(PID_HandleTypeDef* handle, float setpoint, float measurement);

// Conditional anti-windup
float PID_ComputeConditional(PID_HandleTypeDef* handle,
                             float setpoint,
                             float measurement,
                             float outputMin,
                             float outputMax);

// Back-calculation anti-windup
float PID_ComputeBackCalculation(PID_HandleTypeDef* handle,
                                 float setpoint,
                                 float measurement,
                                 float outputMin,
                                 float outputMax,
                                 float antiWindupGain);


#endif /* __PID_H */
