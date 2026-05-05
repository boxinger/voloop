#ifndef VOLOOP_SIGNAL_H
#define VOLOOP_SIGNAL_H

#include "voloop_def.h"

/* ===== Clamp ===== */

float VOLOOP_Signal_Clamp(float value, float min, float max);

VOLOOP_Voltage VOLOOP_Signal_ClampVoltage(VOLOOP_Voltage value, VOLOOP_Voltage min, VOLOOP_Voltage max);

VOLOOP_Current VOLOOP_Signal_ClampCurrent(VOLOOP_Current value, VOLOOP_Current min, VOLOOP_Current max);

VOLOOP_Duty VOLOOP_Signal_ClampDuty(VOLOOP_Duty value, VOLOOP_Duty min, VOLOOP_Duty max);

/* ===== Slew-Rate Limiter ===== */

typedef struct {
    float rateUp;
    float rateDown;
    float output;
} VOLOOP_SlewLimiterTypeDef;

void VOLOOP_Signal_SlewLimiter_Init(VOLOOP_SlewLimiterTypeDef* slew, float rateUp, float rateDown, float initialValue);

float VOLOOP_Signal_SlewLimiter_Update(VOLOOP_SlewLimiterTypeDef* slew, float target, float dt);

/* ===== Linear Ramp Generator ===== */

typedef struct {
    float step;
    float current;
    float target;
} VOLOOP_RampTypeDef;

void VOLOOP_Signal_Ramp_Init(VOLOOP_RampTypeDef* ramp, float step, float startValue);

float VOLOOP_Signal_Ramp_Update(VOLOOP_RampTypeDef* ramp, float target);

void VOLOOP_Signal_Ramp_Reset(VOLOOP_RampTypeDef* ramp, float startValue);

/* ===== Moving Average Filter ===== */

typedef struct {
    float* buffer;
    uint8_t size;
    uint8_t index;
    uint8_t count;
    float sum;
} VOLOOP_MAFTypeDef;

VOLOOP_StatusTypeDef VOLOOP_Signal_MAF_Init(VOLOOP_MAFTypeDef* maf, float* buffer, uint8_t size);

float VOLOOP_Signal_MAF_Update(VOLOOP_MAFTypeDef* maf, float input);

void VOLOOP_Signal_MAF_Reset(VOLOOP_MAFTypeDef* maf);

/* ===== Exponential Moving Average (First-Order IIR Low-Pass) ===== */

typedef struct {
    float alpha;
    float output;
} VOLOOP_EMAFTypeDef;

void VOLOOP_Signal_EMAF_Init(VOLOOP_EMAFTypeDef* ema, float alpha, float initialValue);

float VOLOOP_Signal_EMAF_Update(VOLOOP_EMAFTypeDef* ema, float input);

void VOLOOP_Signal_EMAF_Reset(VOLOOP_EMAFTypeDef* ema, float value);

float VOLOOP_Signal_EMAF_ComputeAlpha(float cutoffFreq, float sampleRate);

#endif /* VOLOOP_SIGNAL_H */
