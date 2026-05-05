#ifndef VOLOOP_SCALE_H
#define VOLOOP_SCALE_H

#include "voloop_def.h"

typedef struct {
    float physMin;
    float physMax;
    float rawMin;
    float rawMax;
} VOLOOP_ScaleTypeDef;

float VOLOOP_Scale_Map(float value, float inMin, float inMax, float outMin, float outMax);

float VOLOOP_Scale_RawToPhys(uint32_t raw, const VOLOOP_ScaleTypeDef* scale);

uint32_t VOLOOP_Scale_PhysToRaw(float phys, const VOLOOP_ScaleTypeDef* scale);

float VOLOOP_Scale_Normalize(float phys, const VOLOOP_ScaleTypeDef* scale);

float VOLOOP_Scale_Denormalize(float norm, const VOLOOP_ScaleTypeDef* scale);

VOLOOP_Voltage VOLOOP_Scale_RawToVoltage(uint32_t raw, const VOLOOP_ScaleTypeDef* scale);

VOLOOP_Current VOLOOP_Scale_RawToCurrent(uint32_t raw, const VOLOOP_ScaleTypeDef* scale);

uint32_t VOLOOP_Scale_DutyToRaw(VOLOOP_Duty duty, const VOLOOP_ScaleTypeDef* scale);

VOLOOP_Duty VOLOOP_Scale_RawToDuty(uint32_t raw, const VOLOOP_ScaleTypeDef* scale);

#endif /* VOLOOP_SCALE_H */
