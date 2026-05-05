#include "voloop_signal.h"

/* ===== Clamp ===== */

float VOLOOP_Signal_Clamp(float value, float min, float max) {
    if (value > max) {
        return max;
    }
    if (value < min) {
        return min;
    }
    return value;
}

VOLOOP_Voltage VOLOOP_Signal_ClampVoltage(VOLOOP_Voltage value, VOLOOP_Voltage min, VOLOOP_Voltage max) {
    return VOLOOP_VOLTAGE(VOLOOP_Signal_Clamp(value.value, min.value, max.value));
}

VOLOOP_Current VOLOOP_Signal_ClampCurrent(VOLOOP_Current value, VOLOOP_Current min, VOLOOP_Current max) {
    return VOLOOP_CURRENT(VOLOOP_Signal_Clamp(value.value, min.value, max.value));
}

VOLOOP_Duty VOLOOP_Signal_ClampDuty(VOLOOP_Duty value, VOLOOP_Duty min, VOLOOP_Duty max) {
    return VOLOOP_DUTY(VOLOOP_Signal_Clamp(value.value, min.value, max.value));
}

/* ===== Slew-Rate Limiter ===== */

void VOLOOP_Signal_SlewLimiter_Init(VOLOOP_SlewLimiterTypeDef* slew, float rateUp, float rateDown, float initialValue) {
    if (slew == NULL) {
        return;
    }
    slew->rateUp = rateUp;
    slew->rateDown = rateDown;
    slew->output = initialValue;
}

float VOLOOP_Signal_SlewLimiter_Update(VOLOOP_SlewLimiterTypeDef* slew, float target, float dt) {
    if (slew == NULL) {
        return 0.0f;
    }
    float maxDelta = slew->rateUp * dt;
    float minDelta = -slew->rateDown * dt;
    float delta = target - slew->output;
    if (delta > maxDelta) {
        delta = maxDelta;
    } else if (delta < minDelta) {
        delta = minDelta;
    }
    slew->output += delta;
    return slew->output;
}

/* ===== Linear Ramp Generator ===== */

void VOLOOP_Signal_Ramp_Init(VOLOOP_RampTypeDef* ramp, float step, float startValue) {
    if (ramp == NULL) {
        return;
    }
    ramp->step = step;
    ramp->current = startValue;
    ramp->target = startValue;
}

float VOLOOP_Signal_Ramp_Update(VOLOOP_RampTypeDef* ramp, float target) {
    if (ramp == NULL) {
        return 0.0f;
    }
    ramp->target = target;
    float error = ramp->target - ramp->current;
    if (error > ramp->step) {
        ramp->current += ramp->step;
    } else if (error < -ramp->step) {
        ramp->current -= ramp->step;
    } else {
        ramp->current = ramp->target;
    }
    return ramp->current;
}

void VOLOOP_Signal_Ramp_Reset(VOLOOP_RampTypeDef* ramp, float startValue) {
    if (ramp == NULL) {
        return;
    }
    ramp->current = startValue;
    ramp->target = startValue;
}

/* ===== Moving Average Filter ===== */

VOLOOP_StatusTypeDef VOLOOP_Signal_MAF_Init(VOLOOP_MAFTypeDef* maf, float* buffer, uint8_t size) {
    if (maf == NULL || buffer == NULL || size == 0) {
        return VOLOOP_INVALID_PARAM;
    }
    maf->buffer = buffer;
    maf->size = size;
    maf->index = 0;
    maf->count = 0;
    maf->sum = 0.0f;
    return VOLOOP_OK;
}

float VOLOOP_Signal_MAF_Update(VOLOOP_MAFTypeDef* maf, float input) {
    if (maf == NULL || maf->buffer == NULL) {
        return 0.0f;
    }
    if (maf->count < maf->size) {
        maf->count++;
    } else {
        maf->sum -= maf->buffer[maf->index];
    }
    maf->buffer[maf->index] = input;
    maf->sum += input;
    maf->index = (maf->index + 1) % maf->size;
    return maf->sum / maf->count;
}

void VOLOOP_Signal_MAF_Reset(VOLOOP_MAFTypeDef* maf) {
    if (maf == NULL) {
        return;
    }
    maf->index = 0;
    maf->count = 0;
    maf->sum = 0.0f;
}

/* ===== Exponential Moving Average ===== */

void VOLOOP_Signal_EMAF_Init(VOLOOP_EMAFTypeDef* ema, float alpha, float initialValue) {
    if (ema == NULL) {
        return;
    }
    if (alpha < 0.0f) {
        alpha = 0.0f;
    }
    if (alpha > 1.0f) {
        alpha = 1.0f;
    }
    ema->alpha = alpha;
    ema->output = initialValue;
}

float VOLOOP_Signal_EMAF_Update(VOLOOP_EMAFTypeDef* ema, float input) {
    if (ema == NULL) {
        return 0.0f;
    }
    ema->output = ema->alpha * input + (1.0f - ema->alpha) * ema->output;
    return ema->output;
}

void VOLOOP_Signal_EMAF_Reset(VOLOOP_EMAFTypeDef* ema, float value) {
    if (ema == NULL) {
        return;
    }
    ema->output = value;
}

float VOLOOP_Signal_EMAF_ComputeAlpha(float cutoffFreq, float sampleRate) {
    if (sampleRate <= 0.0f) {
        return 0.0f;
    }
    float rc = 1.0f / (VOLOOP_TwoPi * cutoffFreq);
    float dt = 1.0f / sampleRate;
    float alpha = dt / (rc + dt);
    if (alpha < 0.0f) {
        alpha = 0.0f;
    }
    if (alpha > 1.0f) {
        alpha = 1.0f;
    }
    return alpha;
}
