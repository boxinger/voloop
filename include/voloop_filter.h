/**
 * @file voloop_filter.h
 * @brief Digital filters for ADC signal conditioning
 *
 * Provides two lightweight filters commonly used in digital power control
 * to reduce ADC quantisation noise before feeding the measured voltage or
 * current into the control loop:
 *
 *  - Moving-average (MA) filter — uniform FIR, configurable window up to
 *    VOLOOP_MA_MAX_SIZE samples.
 *  - First-order IIR low-pass filter — single-coefficient exponential
 *    smoothing: y[n] = α·x[n] + (1−α)·y[n−1].
 *
 * Both filters use only floating-point arithmetic and have no hardware
 * dependencies.
 */

#ifndef VOLOOP_FILTER_H
#define VOLOOP_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** Maximum window length for the moving-average filter. */
#define VOLOOP_MA_MAX_SIZE  16U

/* -------------------------------------------------------------------------
 * Moving-average filter
 * ------------------------------------------------------------------------- */

/**
 * @brief Moving-average filter instance.
 */
typedef struct {
    float   buf[VOLOOP_MA_MAX_SIZE]; /**< Circular sample buffer */
    uint8_t size;                    /**< Configured window length (≤ VOLOOP_MA_MAX_SIZE) */
    uint8_t index;                   /**< Next write position in buf */
    float   sum;                     /**< Running sum of buffered samples */
} voloop_ma_t;

/**
 * @brief Initialise the moving-average filter.
 *
 * @param f     Pointer to the filter instance.
 * @param size  Window length (1 … VOLOOP_MA_MAX_SIZE).
 *              Values outside that range are clamped to VOLOOP_MA_MAX_SIZE.
 */
void voloop_ma_init(voloop_ma_t *f, uint8_t size);

/**
 * @brief Reset filter history to zero without changing the window size.
 * @param f  Pointer to the filter instance.
 */
void voloop_ma_reset(voloop_ma_t *f);

/**
 * @brief Push a new sample and return the updated average.
 *
 * @param f       Pointer to the filter instance.
 * @param sample  New input sample.
 * @return        Current moving average.
 */
float voloop_ma_update(voloop_ma_t *f, float sample);


/* -------------------------------------------------------------------------
 * First-order IIR low-pass filter
 * ------------------------------------------------------------------------- */

/**
 * @brief First-order IIR low-pass filter instance.
 */
typedef struct {
    float alpha;  /**< Smoothing coefficient α ∈ (0, 1].
                   *   α = 1 → no filtering (pass-through).
                   *   Smaller α → heavier filtering / lower cut-off.
                   *
                   *   Approximate cut-off: fc ≈ α·fs / (2π·(1−α))
                   */
    float output; /**< Last filter output (initial state = 0). */
} voloop_iir_t;

/**
 * @brief Initialise the IIR low-pass filter.
 *
 * @param f      Pointer to the filter instance.
 * @param alpha  Smoothing coefficient α ∈ (0, 1].
 */
void voloop_iir_init(voloop_iir_t *f, float alpha);

/**
 * @brief Reset filter state (output → 0) without changing α.
 * @param f  Pointer to the filter instance.
 */
void voloop_iir_reset(voloop_iir_t *f);

/**
 * @brief Filter a new sample.
 *
 * @param f       Pointer to the filter instance.
 * @param sample  New input sample.
 * @return        Filtered output.
 */
float voloop_iir_update(voloop_iir_t *f, float sample);

#ifdef __cplusplus
}
#endif

#endif /* VOLOOP_FILTER_H */
