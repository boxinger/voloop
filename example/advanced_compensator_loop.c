/**
 * @file advanced_compensator_loop.c
 * @brief Type III (3P3Z) compensator + IIR filter voltage loop example
 *
 * Demonstrates a more sophisticated voltage control loop suitable for
 * demanding power supply applications that require high bandwidth and
 * fast transient response:
 *
 *  - A first-order IIR low-pass filter reduces ADC quantisation noise before
 *    the signal enters the compensator.
 *  - A 3P3Z (Type III) compensator provides three poles and three zeros,
 *    giving the 90° phase boost needed to stabilise a power stage whose LC
 *    output filter introduces a double pole.
 *
 * Compensator coefficients must be calculated with a proper design tool
 * (e.g., Microchip DCDT, MATLAB/Octave, or a Python control-design script)
 * and entered here. The values below are illustrative only.
 *
 * On real hardware, replace plant_adc_read() / plant_pwm_write() with the
 * appropriate MCU peripheral calls.
 *
 * Build (host test):
 *   gcc -Wall -Wextra -I../include \
 *       ../src/voloop_compensator.c ../src/voloop_filter.c \
 *       advanced_compensator_loop.c -o advanced_compensator_loop -lm
 */

#include <stdio.h>
#include "voloop.h"

/* ---- Target and plant parameters ---------------------------------------- */

#define VREF      5.0f    /* Target output voltage [V]      */
#define VIN      12.0f    /* Supply voltage [V]             */
#define R_LOAD   10.0f    /* Load resistance [Ω]            */
#define TS        1e-4f   /* Sampling period [s]  (10 kHz)  */
#define SIM_STEPS 500     /* Number of control iterations   */

/* ---- Simplified Buck LC plant (Euler integration) ----------------------- */

#define BUCK_L_H  100e-6f
#define BUCK_C_F  100e-6f

static float s_vout = 0.0f;
static float s_il   = 0.0f;

static float plant_adc_read(void)
{
    return s_vout;
}

static void plant_pwm_write(float duty)
{
    float v_switch;

    if (duty > 1.0f) { duty = 1.0f; }
    if (duty < 0.0f) { duty = 0.0f; }

    v_switch = duty * VIN;
    s_il    += (v_switch - s_vout) / BUCK_L_H * TS;
    if (s_il < 0.0f) { s_il = 0.0f; }
    s_vout  += (s_il - s_vout / R_LOAD) / BUCK_C_F * TS;
}

/* ---- Main --------------------------------------------------------------- */

int main(void)
{
    voloop_3p3z_t comp;
    voloop_iir_t  adc_filter;
    int           step;
    float         vout_raw;
    float         vout_filtered;
    float         error;
    float         duty;

    /*
     * First-order IIR low-pass filter on the ADC reading.
     * alpha = 0.3  →  fc ≈ 0.3 × 10000 / (2π × 0.7) ≈ 682 Hz
     * Adjust alpha to balance noise rejection vs. phase lag.
     */
    voloop_iir_init(&adc_filter, 0.3f);

    /*
     * 3P3Z compensator coefficients.
     *
     * These are example values for the plant above (L = 100 µH, C = 100 µF,
     * R = 10 Ω, Ts = 100 µs).  Derive the correct coefficients for your
     * power stage using a compensator design tool.
     *
     * Transfer function (z-domain):
     *   H(z) = (b0 + b1·z⁻¹ + b2·z⁻² + b3·z⁻³)
     *        / (1  - a1·z⁻¹ - a2·z⁻² - a3·z⁻³)
     */
    voloop_3p3z_init(&comp,
                     /* b0 */  0.20f,
                     /* b1 */ -0.18f,
                     /* b2 */  0.05f,
                     /* b3 */ -0.02f,
                     /* a1 */  1.60f,
                     /* a2 */ -0.70f,
                     /* a3 */  0.10f,
                     /* out_min */ 0.0f,
                     /* out_max */ 1.0f);

    printf("step, vref_V, vout_V, vout_filtered_V, duty\n");

    for (step = 0; step < SIM_STEPS; step++) {
        vout_raw      = plant_adc_read();
        vout_filtered = voloop_iir_update(&adc_filter, vout_raw);

        error = VREF - vout_filtered;
        duty  = voloop_3p3z_update(&comp, error);

        plant_pwm_write(duty);

        if (step % 10 == 0) {
            printf("%4d, %.4f, %.4f, %.4f, %.4f\n",
                   step, VREF, vout_raw, vout_filtered, duty);
        }
    }

    return 0;
}
