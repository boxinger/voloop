#ifndef VOLOOP_OFFINV_H
#define VOLOOP_OFFINV_H

#include "voloop_def.h"
#include "voloop_nco.h"

typedef struct {
    float inputVoltage;
    float outputVoltage;
    float outputCurrent;
} OffInv_InputTypeDef;

typedef struct {
    VOLOOP_DEF_PwmStateTypeDef LeftLegPwmState;
    float LeftLegDuty;
    VOLOOP_DEF_PwmStateTypeDef RightLegPwmState;
    float RightLegDuty;
} OffInv_OutputTypeDef;

typedef struct {
    const NCO_InitTypeDef* NCOInit;
} OffInv_InitTypeDef;

typedef enum {
    OFFINV_ERROR = 0U,
    OFFINV_DISABLED,
    OFFINV_RUNNING,
} OffInv_StateTypeDef;

typedef enum {
    OFFINV_INVALID = 0U,
    OFFINV_NOERROR,
    OFFINV_OCP,
    OFFINV_OVP
} OffInv_FaultCodeTypeDef;

typedef struct {
    OffInv_StateTypeDef State;
    OffInv_FaultCodeTypeDef FaultCode;
    float Duty;
} OffInv_HandleTypeDef;

#define OFFINV_MAX_DUTY 0.90f
#define OFFINV_MIN_DUTY 0.10f

VOLOOP_StatusTypeDef VOLOOP_OffInv_Init(OffInv_HandleTypeDef* handle, OffInv_InitTypeDef* init);
VOLOOP_StatusTypeDef VOLOOP_OffInv_DeInit(OffInv_HandleTypeDef* handle);

VOLOOP_StatusTypeDef VOLOOP_OffInv_Start(OffInv_HandleTypeDef* handle);
VOLOOP_StatusTypeDef VOLOOP_OffInv_Stop(OffInv_HandleTypeDef* handle);
OffInv_StateTypeDef VOLOOP_OffInv_GetState(OffInv_HandleTypeDef* handle);
OffInv_FaultCodeTypeDef VOLOOP_OffInv_GetFaultCode(OffInv_HandleTypeDef* handle);
VOLOOP_StatusTypeDef VOLOOP_OffInv_ClearFaultCode(OffInv_HandleTypeDef* handle);

VOLOOP_StatusTypeDef VOLOOP_OffInv_SetValue(OffInv_HandleTypeDef* handle, float Voltage);
float VOLOOP_OffInv_GetDuty(OffInv_HandleTypeDef* handle);
VOLOOP_StatusTypeDef VOLOOP_OffInv_Sync(OffInv_HandleTypeDef* handle);

#endif