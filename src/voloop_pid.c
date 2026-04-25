/**
 * @file voloop_pid.c
 * @brief Discrete-time PID controller implementation
 */

#include "voloop_pid.h"

void voloop_pid_init(voloop_pid_t *pid,
                     float kp, float ki, float kd,
                     float out_min, float out_max)
{
    pid->kp      = kp;
    pid->ki      = ki;
    pid->kd      = kd;
    pid->out_min = out_min;
    pid->out_max = out_max;
    voloop_pid_reset(pid);
}

void voloop_pid_reset(voloop_pid_t *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
    pid->output     = 0.0f;
}

float voloop_pid_update(voloop_pid_t *pid, float setpoint, float feedback)
{
    float error;
    float p_term;
    float d_term;
    float out;

    error = setpoint - feedback;

    /* Proportional term */
    p_term = pid->kp * error;

    /* Integral term — accumulate then clamp (anti-windup) */
    pid->integral += pid->ki * error;
    if (pid->integral > pid->out_max) {
        pid->integral = pid->out_max;
    } else if (pid->integral < pid->out_min) {
        pid->integral = pid->out_min;
    }

    /* Derivative term (backward difference) */
    d_term          = pid->kd * (error - pid->prev_error);
    pid->prev_error = error;

    /* Sum and saturate output */
    out = p_term + pid->integral + d_term;
    if (out > pid->out_max) {
        out = pid->out_max;
    } else if (out < pid->out_min) {
        out = pid->out_min;
    }

    pid->output = out;
    return out;
}
