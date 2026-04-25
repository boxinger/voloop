/**
 * @file voloop_filter.c
 * @brief Digital filter implementation (moving-average and IIR low-pass)
 */

#include "voloop_filter.h"

/* =========================================================================
 * Moving-average filter
 * ========================================================================= */

void voloop_ma_init(voloop_ma_t *f, uint8_t size)
{
    if ((size == 0U) || (size > VOLOOP_MA_MAX_SIZE)) {
        size = VOLOOP_MA_MAX_SIZE;
    }
    f->size = size;
    voloop_ma_reset(f);
}

void voloop_ma_reset(voloop_ma_t *f)
{
    uint8_t i;

    for (i = 0U; i < VOLOOP_MA_MAX_SIZE; i++) {
        f->buf[i] = 0.0f;
    }
    f->index = 0U;
    f->sum   = 0.0f;
}

float voloop_ma_update(voloop_ma_t *f, float sample)
{
    /* Remove oldest sample from the running sum */
    f->sum -= f->buf[f->index];

    /* Store and add new sample */
    f->buf[f->index] = sample;
    f->sum += sample;

    /* Advance circular index */
    f->index = (uint8_t)((f->index + 1U) % f->size);

    return f->sum / (float)f->size;
}


/* =========================================================================
 * First-order IIR low-pass filter
 * ========================================================================= */

void voloop_iir_init(voloop_iir_t *f, float alpha)
{
    f->alpha = alpha;
    voloop_iir_reset(f);
}

void voloop_iir_reset(voloop_iir_t *f)
{
    f->output = 0.0f;
}

float voloop_iir_update(voloop_iir_t *f, float sample)
{
    f->output = f->alpha * sample + (1.0f - f->alpha) * f->output;
    return f->output;
}
