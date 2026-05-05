#include "voloop_scale.h"

float VOLOOP_Scale_Map(float value, float inMin, float inMax, float outMin, float outMax) {
    if (inMax == inMin) {
        return outMin;
    }
    return outMin + (value - inMin) * (outMax - outMin) / (inMax - inMin);
}

float VOLOOP_Scale_RawToPhys(uint32_t raw, const VOLOOP_ScaleTypeDef* scale) {
    if (scale == NULL) {
        return 0.0f;
    }
    return VOLOOP_Scale_Map((float)raw, scale->rawMin, scale->rawMax, scale->physMin, scale->physMax);
}

uint32_t VOLOOP_Scale_PhysToRaw(float phys, const VOLOOP_ScaleTypeDef* scale) {
    if (scale == NULL) {
        return 0;
    }
    float raw = VOLOOP_Scale_Map(phys, scale->physMin, scale->physMax, scale->rawMin, scale->rawMax);
    if (raw < scale->rawMin) {
        return (uint32_t)scale->rawMin;
    }
    if (raw > scale->rawMax) {
        return (uint32_t)scale->rawMax;
    }
    return (uint32_t)(raw + 0.5f);
}

float VOLOOP_Scale_Normalize(float phys, const VOLOOP_ScaleTypeDef* scale) {
    if (scale == NULL) {
        return 0.0f;
    }
    return VOLOOP_Scale_Map(phys, scale->physMin, scale->physMax, 0.0f, 1.0f);
}

float VOLOOP_Scale_Denormalize(float norm, const VOLOOP_ScaleTypeDef* scale) {
    if (scale == NULL) {
        return 0.0f;
    }
    return VOLOOP_Scale_Map(norm, 0.0f, 1.0f, scale->physMin, scale->physMax);
}

VOLOOP_Voltage VOLOOP_Scale_RawToVoltage(uint32_t raw, const VOLOOP_ScaleTypeDef* scale) {
    return VOLOOP_VOLTAGE(VOLOOP_Scale_RawToPhys(raw, scale));
}

VOLOOP_Current VOLOOP_Scale_RawToCurrent(uint32_t raw, const VOLOOP_ScaleTypeDef* scale) {
    return VOLOOP_CURRENT(VOLOOP_Scale_RawToPhys(raw, scale));
}

uint32_t VOLOOP_Scale_DutyToRaw(VOLOOP_Duty duty, const VOLOOP_ScaleTypeDef* scale) {
    return VOLOOP_Scale_PhysToRaw(duty.value, scale);
}

VOLOOP_Duty VOLOOP_Scale_RawToDuty(uint32_t raw, const VOLOOP_ScaleTypeDef* scale) {
    return VOLOOP_DUTY(VOLOOP_Scale_RawToPhys(raw, scale));
}
