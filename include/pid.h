#ifndef VOLOOP_PID_H
#define VOLOOP_PID_H

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
} PID_InitOneZeroTypeDef;


typedef struct {
    float gain;
    float zero1;
    float zero2;
    uint32_t triggerFrequency;
} PID_InitTwoZeroTypeDef;


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
        PID_InitOneZeroTypeDef OneZero;
        PID_InitTwoZeroTypeDef TwoZero;
    } init;
} PID_InitTypeDef;

/*
#define PID_Init(init) \
    _Generic((init), \
        PID_InitDiscreteTypeDef*: PID_InitDiscrete, \
        PID_InitContinueTypeDef*: PID_InitContinue, \
        PID_InitOneZeroTypeDef*: PID_InitOneZero, \
        PID_InitTwoZeroTypeDef*: PID_InitTwoZero \
    )(init)
*/

PID_HandleTypeDef* VOLOOP_PID_Init(const PID_InitTypeDef* init);
PID_HandleTypeDef* VOLOOP_PID_InitDiscrete(const PID_InitDiscreteTypeDef* init);
PID_HandleTypeDef* VOLOOP_PID_InitContinue(const PID_InitContinueTypeDef* init);
PID_HandleTypeDef* VOLOOP_PID_InitOneZero(const PID_InitOneZeroTypeDef* init);
PID_HandleTypeDef* VOLOOP_PID_InitTwoZero(const PID_InitTwoZeroTypeDef* init);
void VOLOOP_PID_DeInit(PID_HandleTypeDef* handle);
void VOLOOP_PID_Reset(PID_HandleTypeDef* handle);
void VOLOOP_PID_SetIntegral(PID_HandleTypeDef* handle, float integral) ;
void VOLOOP_PID_SetPreviousError(PID_HandleTypeDef* handle, float previousError);
PID_StateTypeDef VOLOOP_PID_GetState(PID_HandleTypeDef* handle);

float VOLOOP_PID_Compute(PID_HandleTypeDef* handle, float setpoint, float measurement);

// Conditional anti-windup
float VOLOOP_PID_ComputeConditional(PID_HandleTypeDef* handle,
                                    float setpoint,
                             float measurement,
                             float outputMin,
                             float outputMax);

// Back-calculation anti-windup
float VOLOOP_PID_ComputeBackCalculation(PID_HandleTypeDef* handle,
                                        float setpoint,
                                        float measurement,
                                        float outputMin,
                                        float outputMax,
                                 float antiWindupGain);


#endif /* __PID_H */
