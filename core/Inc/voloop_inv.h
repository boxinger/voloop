#ifndef VOLOOP_INV_H
#define VOLOOP_INV_H

#include "voloop_def.h"
#include "voloop_nco.h"

typedef struct {
    void (*InitFunc)(void);
    void (*DeInitFunc)(void);
    void (*EnableOutput)(void);
    void (*DisableOutput)(void);
    void (*SetLegADuty)(float duty);
    void (*SetLegBDuty)(float duty);
    const NCO_InitTypeDef* NCOInit;
} Inv_InitTypeDef;

typedef enum{
    INV_ERROR = 0U,
    INV_DISABLED
} Inv_StateTypeDef;

typedef enum{
    INV_INVALID = 0U,
    INV_NOERROR,
    INV_OCP,
    INV_OVP
}Inv_FaultCodeTypeDef;

typedef struct Inv_HandleTypeDef Inv_HandleTypeDef;


#define INV_MAX_DUTY 0.90f
#define INV_MIN_DUTY 0.10f

VOLOOP_StatusTypeDef VOLOOP_Inv_Init(Inv_HandleTypeDef** handleOut, Inv_InitTypeDef* init);
VOLOOP_StatusTypeDef VOLOOP_Inv_DeInit(Inv_HandleTypeDef** handleOut);

VOLOOP_StatusTypeDef VOLOOP_Inv_Start(Inv_HandleTypeDef* handle);
VOLOOP_StatusTypeDef VOLOOP_Inv_Stop(Inv_HandleTypeDef* handle);
Inv_StateTypeDef VOLOOP_Inv_GetState(Inv_HandleTypeDef* handle);
Inv_FaultCodeTypeDef VOLOOP_Inv_GetFaultCode(Inv_HandleTypeDef* handle);
VOLOOP_StatusTypeDef VOLOOP_Inv_ClearFaultCode(Inv_HandleTypeDef* handle);

VOLOOP_StatusTypeDef VOLOOP_Inv_SetValue(Inv_HandleTypeDef* handle, float Voltage);
float VOLOOP_Inv_GetDuty(Inv_HandleTypeDef* handle);
VOLOOP_StatusTypeDef VOLOOP_Inv_Sync(Inv_HandleTypeDef* handle);

#endif