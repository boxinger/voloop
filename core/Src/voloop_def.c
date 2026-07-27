#include "voloop_def.h"

#define VOLOOP_DEF_Q31_SCALE 2147483648.0f
#define VOLOOP_DEF_Q31_SCALE_INV (1.0f / VOLOOP_DEF_Q31_SCALE)
#define VOLOOP_DEF_SIN_TABLE_BITS 10U
#define VOLOOP_DEF_SIN_TABLE_SIZE (1U << VOLOOP_DEF_SIN_TABLE_BITS)
#define VOLOOP_DEF_SIN_TABLE_MASK (VOLOOP_DEF_SIN_TABLE_SIZE - 1U)
#define VOLOOP_DEF_SIN_FRAC_BITS (32U - VOLOOP_DEF_SIN_TABLE_BITS)
#define VOLOOP_DEF_SIN_FRAC_MASK ((1UL << VOLOOP_DEF_SIN_FRAC_BITS) - 1UL)
#define VOLOOP_DEF_SIN_FRAC_SCALE_INV (1.0f / (float)(1UL << VOLOOP_DEF_SIN_FRAC_BITS))
#define VOLOOP_DEF_Q31_QUARTER_CYCLE 0x40000000UL
#define VOLOOP_DEF_ONE_THIRD 0.33333333333333333333f
#define VOLOOP_DEF_ONE_OVER_SQRT3 0.57735026918962576451f
#define VOLOOP_DEF_SQRT3_OVER_TWO 0.86602540378443864676f

static const float s_voLoopSinTable[VOLOOP_DEF_SIN_TABLE_SIZE] = {
#include "voloop_sin_table_1024.inc"
};

static float VOLOOP_DEF_LookupSinCycle(uint32_t phaseCycleU32) {
    uint32_t index0 = phaseCycleU32 >> VOLOOP_DEF_SIN_FRAC_BITS;
    uint32_t index1 = (index0 + 1U) & VOLOOP_DEF_SIN_TABLE_MASK;
    uint32_t fracPart = phaseCycleU32 & VOLOOP_DEF_SIN_FRAC_MASK;
    float y0 = s_voLoopSinTable[index0];
    float y1 = s_voLoopSinTable[index1];
    float frac = (float)fracPart * VOLOOP_DEF_SIN_FRAC_SCALE_INV;

    return y0 + (frac * (y1 - y0));
}

float VOLOOP_DEF_ClampFloat(float value, float min, float max) {
    if (value < min) {
        return min;
    } else if (value > max) {
        return max;
    } else {
        return value;
    }
}

float VOLOOP_DEF_CalcAlphaByCycles(float triggerFrequency,
                                   float nominalFrequency,
                                   float trackCycles) {
    if (triggerFrequency <= 0.0f ||
        nominalFrequency <= 0.0f ||
        trackCycles <= 0.0f) {
        return 0.0f;
    }

    return nominalFrequency / (triggerFrequency * trackCycles);
}

float VOLOOP_DEF_Q31ToRad(int32_t value) {
    return (float)value * VOLOOP_Pi * VOLOOP_DEF_Q31_SCALE_INV;
}

int32_t VOLOOP_DEF_RadToQ31(float value) {
    if (value <= -VOLOOP_Pi) {
        return INT32_MIN;
    }

    if (value >= VOLOOP_Pi) {
        return INT32_MAX;
    }

    return (int32_t)(value * VOLOOP_DEF_Q31_SCALE * VOLOOP_Pi_Inv);
}

float VOLOOP_DEF_SINQ31(int32_t phaseQ31) {
    return VOLOOP_DEF_LookupSinCycle((uint32_t)phaseQ31);
}

float VOLOOP_DEF_COSQ31(int32_t phaseQ31) {
    return VOLOOP_DEF_LookupSinCycle((uint32_t)phaseQ31 + VOLOOP_DEF_Q31_QUARTER_CYCLE);
}

void VOLOOP_DEF_ClarkeTransform(const VOLOOP_DEF_AbcTypeDef* input,
                                VOLOOP_DEF_AlphaBetaZeroTypeDef* output) {
    output->alpha =
        ((2.0f * input->a) - input->b - input->c) * VOLOOP_DEF_ONE_THIRD;
    output->beta = (input->b - input->c) * VOLOOP_DEF_ONE_OVER_SQRT3;
    output->zero = (input->a + input->b + input->c) * VOLOOP_DEF_ONE_THIRD;
}

void VOLOOP_DEF_InverseClarkeTransform(const VOLOOP_DEF_AlphaBetaZeroTypeDef* input,
                                       VOLOOP_DEF_AbcTypeDef* output) {
    output->a = input->alpha + input->zero;
    output->b =
        (-0.5f * input->alpha) + (VOLOOP_DEF_SQRT3_OVER_TWO * input->beta) + input->zero;
    output->c =
        (-0.5f * input->alpha) - (VOLOOP_DEF_SQRT3_OVER_TWO * input->beta) + input->zero;
}

void VOLOOP_DEF_ParkTransform(const VOLOOP_DEF_AlphaBetaZeroTypeDef* input,
                              int32_t phaseQ31,
                              VOLOOP_DEF_DqZeroTypeDef* output) {
    float sine = VOLOOP_DEF_SIN(phaseQ31);
    float cosine = VOLOOP_DEF_COS(phaseQ31);

    output->d = (input->alpha * cosine) + (input->beta * sine);
    output->q = (-input->alpha * sine) + (input->beta * cosine);
    output->zero = input->zero;
}

void VOLOOP_DEF_InverseParkTransform(const VOLOOP_DEF_DqZeroTypeDef* input,
                                     int32_t phaseQ31,
                                     VOLOOP_DEF_AlphaBetaZeroTypeDef* output) {
    float sine = VOLOOP_DEF_SIN(phaseQ31);
    float cosine = VOLOOP_DEF_COS(phaseQ31);

    output->alpha = (input->d * cosine) - (input->q * sine);
    output->beta = (input->d * sine) + (input->q * cosine);
    output->zero = input->zero;
}
