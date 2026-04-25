/**
 * @file voloop_compensator.c
 * @brief Type II (2P2Z) and Type III (3P3Z) digital compensator implementation
 *
 * Both compensators use a Direct Form I realization which avoids coefficient
 * sensitivity problems present in Direct Form II for higher-order sections.
 */

#include "voloop_compensator.h"

/* =========================================================================
 * 2P2Z — 2-Pole 2-Zero (Type II)
 * ========================================================================= */

void voloop_2p2z_init(voloop_2p2z_t *comp,
                      float b0, float b1, float b2,
                      float a1, float a2,
                      float out_min, float out_max)
{
    comp->b0 = b0;
    comp->b1 = b1;
    comp->b2 = b2;
    comp->a1 = a1;
    comp->a2 = a2;
    comp->out_min = out_min;
    comp->out_max = out_max;
    voloop_2p2z_reset(comp);
}

void voloop_2p2z_reset(voloop_2p2z_t *comp)
{
    comp->x1 = 0.0f;
    comp->x2 = 0.0f;
    comp->y1 = 0.0f;
    comp->y2 = 0.0f;
}

float voloop_2p2z_update(voloop_2p2z_t *comp, float error)
{
    float y;

    /* Direct Form I difference equation */
    y = comp->b0 * error
      + comp->b1 * comp->x1
      + comp->b2 * comp->x2
      + comp->a1 * comp->y1
      + comp->a2 * comp->y2;

    /* Saturate output */
    if (y > comp->out_max) {
        y = comp->out_max;
    } else if (y < comp->out_min) {
        y = comp->out_min;
    }

    /* Shift history */
    comp->x2 = comp->x1;
    comp->x1 = error;
    comp->y2 = comp->y1;
    comp->y1 = y;

    return y;
}


/* =========================================================================
 * 3P3Z — 3-Pole 3-Zero (Type III)
 * ========================================================================= */

void voloop_3p3z_init(voloop_3p3z_t *comp,
                      float b0, float b1, float b2, float b3,
                      float a1, float a2, float a3,
                      float out_min, float out_max)
{
    comp->b0 = b0;
    comp->b1 = b1;
    comp->b2 = b2;
    comp->b3 = b3;
    comp->a1 = a1;
    comp->a2 = a2;
    comp->a3 = a3;
    comp->out_min = out_min;
    comp->out_max = out_max;
    voloop_3p3z_reset(comp);
}

void voloop_3p3z_reset(voloop_3p3z_t *comp)
{
    comp->x1 = 0.0f;
    comp->x2 = 0.0f;
    comp->x3 = 0.0f;
    comp->y1 = 0.0f;
    comp->y2 = 0.0f;
    comp->y3 = 0.0f;
}

float voloop_3p3z_update(voloop_3p3z_t *comp, float error)
{
    float y;

    /* Direct Form I difference equation */
    y = comp->b0 * error
      + comp->b1 * comp->x1
      + comp->b2 * comp->x2
      + comp->b3 * comp->x3
      + comp->a1 * comp->y1
      + comp->a2 * comp->y2
      + comp->a3 * comp->y3;

    /* Saturate output */
    if (y > comp->out_max) {
        y = comp->out_max;
    } else if (y < comp->out_min) {
        y = comp->out_min;
    }

    /* Shift history */
    comp->x3 = comp->x2;
    comp->x2 = comp->x1;
    comp->x1 = error;
    comp->y3 = comp->y2;
    comp->y2 = comp->y1;
    comp->y1 = y;

    return y;
}
