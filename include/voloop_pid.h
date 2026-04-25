/**
 * @file voloop_pid.h
 * @brief Discrete-time PID controller for digital power control
 *
 * Implements the standard position-form PID with:
 *   - Configurable proportional, integral, and derivative gains
 *   - Clamped integral anti-windup
 *   - Output saturation limiting
 */

#ifndef VOLOOP_PID_H
#define VOLOOP_PID_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief PID controller instance
 */
typedef struct {
    float kp;           /**< Proportional gain */
    float ki;           /**< Integral gain (already scaled by Ts) */
    float kd;           /**< Derivative gain (already scaled by 1/Ts) */
    float out_max;      /**< Upper output limit */
    float out_min;      /**< Lower output limit */
    float integral;     /**< Integral accumulator */
    float prev_error;   /**< Error from previous update (for derivative) */
    float output;       /**< Most recent controller output */
} voloop_pid_t;

/**
 * @brief Initialise a PID controller.
 *
 * @param pid      Pointer to the PID instance to initialise.
 * @param kp       Proportional gain.
 * @param ki       Integral gain (pre-multiplied by the sampling period Ts).
 * @param kd       Derivative gain (pre-divided by the sampling period Ts).
 * @param out_min  Minimum controller output (e.g. 0.0 for duty cycle).
 * @param out_max  Maximum controller output (e.g. 1.0 for duty cycle).
 */
void voloop_pid_init(voloop_pid_t *pid,
                     float kp, float ki, float kd,
                     float out_min, float out_max);

/**
 * @brief Reset controller state without changing tuning parameters.
 *
 * @param pid  Pointer to the PID instance.
 */
void voloop_pid_reset(voloop_pid_t *pid);

/**
 * @brief Run one control iteration.
 *
 * Must be called at a constant rate equal to the sampling period used when
 * tuning the gains.
 *
 * @param pid       Pointer to the PID instance.
 * @param setpoint  Desired (reference) value.
 * @param feedback  Measured (plant output) value.
 * @return          Saturated controller output.
 */
float voloop_pid_update(voloop_pid_t *pid, float setpoint, float feedback);

#ifdef __cplusplus
}
#endif

#endif /* VOLOOP_PID_H */
