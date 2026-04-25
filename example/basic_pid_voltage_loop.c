/**
 * @file basic_pid_voltage_loop.c
 * @brief Basic PID voltage control loop example — Buck converter simulation
 *
 * Demonstrates using voloop_pid to regulate the output voltage of a
 * synchronous Buck converter.
 *
 * On real hardware replace plant_adc_read() / plant_pwm_write() with your
 * MCU's ADC and PWM peripheral calls. The voloop_pid API is identical in
 * both cases — only the I/O stubs change.
 *
 * Control loop (10 kHz, Ts = 100 µs):
 *   1. Read Vout via ADC
 *   2. Compute error = Vref - Vout
 *   3. Run PID → duty cycle
 *   4. Write duty cycle to PWM timer
 *
 * Build (host test):
 *   gcc -Wall -Wextra -I../include ../src/voloop_pid.c \
 *       basic_pid_voltage_loop.c -o basic_pid_voltage_loop -lm
 */

#include <stdio.h>
#include "voloop.h"

/* ---- Target and plant parameters ---------------------------------------- */

#define VREF      5.0f    /* Target output voltage [V]       */
#define VIN      12.0f    /* Supply voltage [V]              */
#define R_LOAD   10.0f    /* Load resistance [Ω]             */
#define TS        1e-4f   /* Sampling period [s]  (10 kHz)   */
#define SIM_STEPS 500     /* Number of control iterations    */

/* ---- Simplified Buck LC plant (Euler integration) ----------------------- */

#define BUCK_L_H  100e-6f   /* Inductance  [H]  */
#define BUCK_C_F  100e-6f   /* Capacitance [F]  */

static float s_vout = 0.0f;  /* Simulated output voltage */
static float s_il   = 0.0f;  /* Simulated inductor current */

/** Simulate an ADC reading of the output voltage. */
static float plant_adc_read(void)
{
    return s_vout;
}

/** Simulate writing a duty cycle to the PWM peripheral. */
static void plant_pwm_write(float duty)
{
    float v_switch;

    /* Clamp duty to [0, 1] as a real PWM would */
    if (duty > 1.0f) { duty = 1.0f; }
    if (duty < 0.0f) { duty = 0.0f; }

    /* Forward-Euler integration of ideal Buck LC dynamics */
    v_switch = duty * VIN;
    s_il    += (v_switch - s_vout) / BUCK_L_H * TS;
    if (s_il < 0.0f) { s_il = 0.0f; }          /* diode free-wheeling */
    s_vout  += (s_il - s_vout / R_LOAD) / BUCK_C_F * TS;
}

/* ---- Main --------------------------------------------------------------- */

int main(void)
{
    voloop_pid_t pid;
    int          step;
    float        vout;
    float        duty;

    /*
     * PID gains tuned for this plant at Ts = 100 µs.
     * ki is pre-scaled by Ts; kd is pre-scaled by 1/Ts.
     */
    voloop_pid_init(&pid,
                    /* kp      */ 0.15f,
                    /* ki      */ 0.08f,
                    /* kd      */ 0.001f,
                    /* out_min */ 0.0f,
                    /* out_max */ 1.0f);

    printf("step, vref_V, vout_V, duty\n");

    for (step = 0; step < SIM_STEPS; step++) {
        vout = plant_adc_read();
        duty = voloop_pid_update(&pid, VREF, vout);
        plant_pwm_write(duty);

        if (step % 10 == 0) {
            printf("%4d, %.4f, %.4f, %.4f\n", step, VREF, vout, duty);
        }
    }

    return 0;
}
