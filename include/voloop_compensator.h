/**
 * @file voloop_compensator.h
 * @brief Type II (2P2Z) and Type III (3P3Z) digital compensators
 *
 * These are the industry-standard discrete-time compensators used in
 * voltage-mode and current-mode digital power control loops.
 *
 * Both structures implement a Direct Form I difference equation:
 *
 *   2P2Z:  y[n] = b0*e[n] + b1*e[n-1] + b2*e[n-2]
 *                          + a1*y[n-1] + a2*y[n-2]
 *
 *   3P3Z:  y[n] = b0*e[n] + b1*e[n-1] + b2*e[n-2] + b3*e[n-3]
 *                          + a1*y[n-1] + a2*y[n-2] + a3*y[n-3]
 *
 * Coefficients are derived from the desired s-domain compensator using a
 * bilinear (Tustin) or matched-pole-zero transformation at the chosen
 * switching/sampling frequency.
 */

#ifndef VOLOOP_COMPENSATOR_H
#define VOLOOP_COMPENSATOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * 2-Pole 2-Zero (Type II) compensator
 * ------------------------------------------------------------------------- */

/**
 * @brief 2P2Z compensator instance.
 */
typedef struct {
    float b0, b1, b2;   /**< Numerator (zero) coefficients */
    float a1, a2;       /**< Denominator (pole) coefficients */
    float out_max;      /**< Output upper limit */
    float out_min;      /**< Output lower limit */
    /* History */
    float x1, x2;      /**< Previous two error inputs */
    float y1, y2;      /**< Previous two outputs */
} voloop_2p2z_t;

/**
 * @brief Initialise a 2P2Z compensator.
 *
 * @param comp     Pointer to the compensator instance.
 * @param b0..b2   Numerator coefficients.
 * @param a1..a2   Denominator coefficients.
 * @param out_min  Minimum clamped output.
 * @param out_max  Maximum clamped output.
 */
void voloop_2p2z_init(voloop_2p2z_t *comp,
                      float b0, float b1, float b2,
                      float a1, float a2,
                      float out_min, float out_max);

/**
 * @brief Reset compensator history (states) without changing coefficients.
 * @param comp  Pointer to the compensator instance.
 */
void voloop_2p2z_reset(voloop_2p2z_t *comp);

/**
 * @brief Run one 2P2Z update.
 *
 * @param comp   Pointer to the compensator instance.
 * @param error  Current loop error (reference − feedback).
 * @return       Clamped compensator output.
 */
float voloop_2p2z_update(voloop_2p2z_t *comp, float error);


/* -------------------------------------------------------------------------
 * 3-Pole 3-Zero (Type III) compensator
 * ------------------------------------------------------------------------- */

/**
 * @brief 3P3Z compensator instance.
 */
typedef struct {
    float b0, b1, b2, b3;  /**< Numerator (zero) coefficients */
    float a1, a2, a3;      /**< Denominator (pole) coefficients */
    float out_max;         /**< Output upper limit */
    float out_min;         /**< Output lower limit */
    /* History */
    float x1, x2, x3;     /**< Previous three error inputs */
    float y1, y2, y3;     /**< Previous three outputs */
} voloop_3p3z_t;

/**
 * @brief Initialise a 3P3Z compensator.
 *
 * @param comp     Pointer to the compensator instance.
 * @param b0..b3   Numerator coefficients.
 * @param a1..a3   Denominator coefficients.
 * @param out_min  Minimum clamped output.
 * @param out_max  Maximum clamped output.
 */
void voloop_3p3z_init(voloop_3p3z_t *comp,
                      float b0, float b1, float b2, float b3,
                      float a1, float a2, float a3,
                      float out_min, float out_max);

/**
 * @brief Reset compensator history (states) without changing coefficients.
 * @param comp  Pointer to the compensator instance.
 */
void voloop_3p3z_reset(voloop_3p3z_t *comp);

/**
 * @brief Run one 3P3Z update.
 *
 * @param comp   Pointer to the compensator instance.
 * @param error  Current loop error (reference − feedback).
 * @return       Clamped compensator output.
 */
float voloop_3p3z_update(voloop_3p3z_t *comp, float error);

#ifdef __cplusplus
}
#endif

#endif /* VOLOOP_COMPENSATOR_H */
