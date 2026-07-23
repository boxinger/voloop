#include "voloop_pfc.h"
#include "voloop_test.h"
#include "voloop_test_float.h"

#include <math.h>

#define TEST_TOLERANCE 1.0e-4f
#define TEST_TRIGGER_FREQUENCY 10000.0f

#define TEST_EXPECT_STATUS_EQ(ctx, expected, actual)                                               \
    VOLOOP_EXPECT_EQ_INT((ctx), (expected), (actual))
#define TEST_REQUIRE_STATUS_EQ(ctx, expected, actual)                                              \
    VOLOOP_REQUIRE_EQ_INT((ctx), (expected), (actual))
#define TEST_EXPECT_STATE_EQ(ctx, expected, actual)                                                \
    VOLOOP_EXPECT_EQ_INT((ctx), (expected), (actual))
#define TEST_EXPECT_FAULT_EQ(ctx, expected, actual)                                                \
    VOLOOP_EXPECT_EQ_INT((ctx), (expected), (actual))
#define TEST_EXPECT_FLOAT_NEAR(ctx, expected, actual)                                              \
    VOLOOP_EXPECT_FLOAT_NEAR((ctx), (expected), (actual), TEST_TOLERANCE)

static PID_InitTypeDef make_pid_init(float kp, float ki) {
    PID_InitTypeDef init = { 0 };

    init.mode = PID_Discrete;
    init.init.Discrete.KpDiscrete = kp;
    init.init.Discrete.KiDiscrete = ki;
    init.init.Discrete.KdDiscrete = 0.0f;

    return init;
}

static NCO_InitTypeDef make_nco_init(void) {
    NCO_InitTypeDef init = {
        .triggerFrequency = (uint32_t)TEST_TRIGGER_FREQUENCY,
        .initialFrequency = 50.0f,
        .initialRad = 0.0f,
    };

    return init;
}

static PLL_InitTypeDef make_pll_init(const PID_InitTypeDef* loop_pid, const NCO_InitTypeDef* nco) {
    PLL_InitTypeDef init = {
        .LoopFilterInit = loop_pid,
        .NCOInit = nco,
        .triggerFrequency = TEST_TRIGGER_FREQUENCY,
    };

    return init;
}

static PFC_ConfigTypeDef make_pfc_config(void) {
    PFC_ConfigTypeDef config = {
        .BusOverVoltageThreshold = 450.0f,
        .GridOverCurrentThreshold = 30.0f,
        .MinHighFrequencyDuty = 0.05f,
        .MaxHighFrequencyDuty = 0.95f,
        .ZeroCrossingDeadband = 0.05f,
    };

    return config;
}

static PFC_InitTypeDef make_pfc_init(const PLL_InitTypeDef* pll, const PID_InitTypeDef* bus_pid,
                                     const PID_InitTypeDef* current_pid,
                                     const PFC_ConfigTypeDef* config) {
    PFC_InitTypeDef init = {
        .PLLInit = pll,
        .BusVoltagePIDInit = bus_pid,
        .GridCurrentPIDInit = current_pid,
        .Config = config,
        .TriggerFrequency = TEST_TRIGGER_FREQUENCY,
    };

    return init;
}

static PFC_HandleTypeDef make_started_pfc(float bus_kp, float bus_ki, float current_kp,
                                          float current_ki) {
    PFC_HandleTypeDef pfc = { 0 };
    PID_InitTypeDef pll_pid = make_pid_init(0.0f, 0.0f);
    PID_InitTypeDef bus_pid = make_pid_init(bus_kp, bus_ki);
    PID_InitTypeDef current_pid = make_pid_init(current_kp, current_ki);
    NCO_InitTypeDef nco = make_nco_init();
    PLL_InitTypeDef pll = make_pll_init(&pll_pid, &nco);
    PFC_ConfigTypeDef config = make_pfc_config();
    PFC_InitTypeDef init = make_pfc_init(&pll, &bus_pid, &current_pid, &config);

    (void)VOLOOP_PFC_Init(&pfc, &init);
    (void)VOLOOP_PFC_SetValue(&pfc, 400.0f, 20.0f);
    (void)VOLOOP_PFC_Start(&pfc);

    return pfc;
}

static void force_phase_for_next_sync(PFC_HandleTypeDef* pfc, float rad) {
    uint32_t target_phase = (uint32_t)VOLOOP_DEF_RadToQ31(rad);
    uint32_t phase_step = pfc->PLL.NCO.PhaseStepQ31;

    pfc->PLL.NCO.PhaseQ31 = (int32_t)(target_phase - phase_step);
    pfc->PLL.PhaseQ31 = pfc->PLL.NCO.PhaseQ31;
    pfc->PLL.LockState = PLL_LOCKED;
    pfc->PLL.LockCounter = 0U;
    pfc->PLL.UnlockCounter = 0U;
}

static void expect_output_disabled(VoloopTestContext* ctx, const PFC_OutputTypeDef* output) {
    TEST_EXPECT_STATE_EQ(ctx, VOLOOP_PWM_DISABLED, output->LeftLegPwmState);
    TEST_EXPECT_FLOAT_NEAR(ctx, 0.0f, output->LeftLegDuty);
    TEST_EXPECT_STATE_EQ(ctx, VOLOOP_PWM_DISABLED, output->RightLegPwmState);
    TEST_EXPECT_FLOAT_NEAR(ctx, 0.0f, output->RightLegDuty);
}

static void test_voltage_loop_generates_current_amplitude_limit(VoloopTestContext* ctx) {
    PFC_HandleTypeDef pfc = make_started_pfc(1.0f, 0.0f, 0.01f, 0.0f);
    PFC_OutputTypeDef output = { 0 };
    PFC_InputTypeDef input = {
        .GridVoltage = 100.0f,
        .GridCurrent = 0.0f,
        .BusVoltage = 300.0f,
    };
    force_phase_for_next_sync(&pfc, VOLOOP_Pi * 0.5f);

    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_Sync(&pfc, &input, &output));

    TEST_EXPECT_STATE_EQ(ctx, PFC_CURRENT_LIMIT, VOLOOP_PFC_GetState(&pfc));
    TEST_EXPECT_FAULT_EQ(ctx, PFC_FAULT_NONE, VOLOOP_PFC_GetFaultCode(&pfc));
    TEST_EXPECT_FLOAT_NEAR(ctx, 20.0f, VOLOOP_PFC_GetCurrentAmplitudeReference(&pfc));
    TEST_EXPECT_FLOAT_NEAR(ctx, 20.0f, VOLOOP_PFC_GetCurrentReference(&pfc));
}

static void test_positive_and_negative_bridge_mapping(VoloopTestContext* ctx) {
    PFC_HandleTypeDef positive = make_started_pfc(1.0f, 0.0f, 0.01f, 0.0f);
    PFC_HandleTypeDef negative = make_started_pfc(1.0f, 0.0f, 0.01f, 0.0f);
    PFC_OutputTypeDef output = { 0 };
    PFC_InputTypeDef input = {
        .GridVoltage = 100.0f,
        .GridCurrent = 0.0f,
        .BusVoltage = 390.0f,
    };

    force_phase_for_next_sync(&positive, VOLOOP_Pi * 0.5f);
    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_Sync(&positive, &input, &output));
    TEST_EXPECT_STATE_EQ(ctx, PFC_LINE_POSITIVE, VOLOOP_PFC_GetLinePolarity(&positive));
    TEST_EXPECT_FLOAT_NEAR(ctx, 10.0f, VOLOOP_PFC_GetCurrentReference(&positive));
    TEST_EXPECT_FLOAT_NEAR(ctx, 0.1f, VOLOOP_PFC_GetModulation(&positive));
    TEST_EXPECT_STATE_EQ(ctx, VOLOOP_PWM_ENABLE, output.LeftLegPwmState);
    TEST_EXPECT_FLOAT_NEAR(ctx, 0.9f, output.LeftLegDuty);
    TEST_EXPECT_STATE_EQ(ctx, VOLOOP_PWM_ENABLE, output.RightLegPwmState);
    TEST_EXPECT_FLOAT_NEAR(ctx, 0.0f, output.RightLegDuty);

    input.GridVoltage = -100.0f;
    force_phase_for_next_sync(&negative, -VOLOOP_Pi * 0.5f);
    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_Sync(&negative, &input, &output));
    TEST_EXPECT_STATE_EQ(ctx, PFC_LINE_NEGATIVE, VOLOOP_PFC_GetLinePolarity(&negative));
    TEST_EXPECT_FLOAT_NEAR(ctx, -10.0f, VOLOOP_PFC_GetCurrentReference(&negative));
    TEST_EXPECT_FLOAT_NEAR(ctx, -0.1f, VOLOOP_PFC_GetModulation(&negative));
    TEST_EXPECT_STATE_EQ(ctx, VOLOOP_PWM_ENABLE, output.LeftLegPwmState);
    TEST_EXPECT_FLOAT_NEAR(ctx, 0.0f, output.LeftLegDuty);
    TEST_EXPECT_STATE_EQ(ctx, VOLOOP_PWM_ENABLE, output.RightLegPwmState);
    TEST_EXPECT_FLOAT_NEAR(ctx, 0.9f, output.RightLegDuty);
}

static void test_negative_half_cycle_aligns_current_measurement(VoloopTestContext* ctx) {
    PFC_HandleTypeDef pfc = make_started_pfc(1.0f, 0.0f, 0.01f, 0.0f);
    PFC_OutputTypeDef output = { 0 };
    PFC_InputTypeDef input = {
        .GridVoltage = -100.0f,
        .GridCurrent = -5.0f,
        .BusVoltage = 390.0f,
    };
    force_phase_for_next_sync(&pfc, -VOLOOP_Pi * 0.5f);

    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_Sync(&pfc, &input, &output));

    TEST_EXPECT_FLOAT_NEAR(ctx, -0.05f, VOLOOP_PFC_GetModulation(&pfc));
    TEST_EXPECT_FLOAT_NEAR(ctx, 0.95f, output.RightLegDuty);
}

static void test_zero_crossing_resets_current_controller(VoloopTestContext* ctx) {
    PFC_HandleTypeDef pfc = make_started_pfc(1.0f, 0.0f, 0.01f, 0.01f);
    PFC_OutputTypeDef output = { 0 };
    PFC_InputTypeDef input = {
        .GridVoltage = 0.0f,
        .GridCurrent = 0.0f,
        .BusVoltage = 390.0f,
    };

    pfc.GridCurrentPID.Integral = 3.0f;
    pfc.GridCurrentPID.PreviousError = -2.0f;
    pfc.GridCurrentPID.State = PID_UpperSaturated;
    pfc.Modulation = 0.4f;
    force_phase_for_next_sync(&pfc, 0.0f);

    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_Sync(&pfc, &input, &output));

    TEST_EXPECT_STATE_EQ(ctx, PFC_LINE_ZERO, VOLOOP_PFC_GetLinePolarity(&pfc));
    TEST_EXPECT_FLOAT_NEAR(ctx, 0.0f, pfc.GridCurrentPID.Integral);
    TEST_EXPECT_FLOAT_NEAR(ctx, 0.0f, pfc.GridCurrentPID.PreviousError);
    TEST_EXPECT_STATE_EQ(ctx, PID_UnSaturated, pfc.GridCurrentPID.State);
    TEST_EXPECT_FLOAT_NEAR(ctx, 0.0f, VOLOOP_PFC_GetModulation(&pfc));
    expect_output_disabled(ctx, &output);
}

static void test_high_frequency_duty_uses_configured_limits(VoloopTestContext* ctx) {
    PFC_HandleTypeDef max_duty = make_started_pfc(1.0f, 0.0f, 0.001f, 0.0f);
    PFC_HandleTypeDef min_duty = make_started_pfc(1.0f, 0.0f, 1.0f, 0.0f);
    PFC_OutputTypeDef output = { 0 };
    PFC_InputTypeDef input = {
        .GridVoltage = 100.0f,
        .GridCurrent = 0.0f,
        .BusVoltage = 390.0f,
    };

    force_phase_for_next_sync(&max_duty, VOLOOP_Pi * 0.5f);
    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_Sync(&max_duty, &input, &output));
    TEST_EXPECT_FLOAT_NEAR(ctx, 0.95f, output.LeftLegDuty);

    force_phase_for_next_sync(&min_duty, VOLOOP_Pi * 0.5f);
    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_Sync(&min_duty, &input, &output));
    TEST_EXPECT_FLOAT_NEAR(ctx, 1.0f, VOLOOP_PFC_GetModulation(&min_duty));
    TEST_EXPECT_FLOAT_NEAR(ctx, 0.05f, output.LeftLegDuty);
}

static void test_active_protection_latches_fault_and_stops_pll(VoloopTestContext* ctx) {
    PFC_HandleTypeDef over_current = make_started_pfc(1.0f, 0.0f, 0.01f, 0.0f);
    PFC_HandleTypeDef over_voltage = make_started_pfc(1.0f, 0.0f, 0.01f, 0.0f);
    PFC_HandleTypeDef invalid_sample = make_started_pfc(1.0f, 0.0f, 0.01f, 0.0f);
    PFC_OutputTypeDef output = { 0 };
    PFC_InputTypeDef input = {
        .GridVoltage = 100.0f,
        .GridCurrent = 31.0f,
        .BusVoltage = 390.0f,
    };

    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_ERROR, VOLOOP_PFC_Sync(&over_current, &input, &output));
    TEST_EXPECT_FAULT_EQ(ctx, PFC_FAULT_INPUT_OVERCURRENT, VOLOOP_PFC_GetFaultCode(&over_current));
    TEST_EXPECT_STATE_EQ(ctx, PLL_STOPPED, VOLOOP_PLL_GetState(&over_current.PLL));
    expect_output_disabled(ctx, &output);

    input.GridCurrent = 0.0f;
    input.BusVoltage = 451.0f;
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_ERROR, VOLOOP_PFC_Sync(&over_voltage, &input, &output));
    TEST_EXPECT_FAULT_EQ(ctx, PFC_FAULT_BUS_OVERVOLTAGE, VOLOOP_PFC_GetFaultCode(&over_voltage));
    TEST_EXPECT_STATE_EQ(ctx, PLL_STOPPED, VOLOOP_PLL_GetState(&over_voltage.PLL));

    input.BusVoltage = NAN;
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_ERROR, VOLOOP_PFC_Sync(&invalid_sample, &input, &output));
    TEST_EXPECT_FAULT_EQ(ctx, PFC_FAULT_INVALID_SAMPLE, VOLOOP_PFC_GetFaultCode(&invalid_sample));
    TEST_EXPECT_STATE_EQ(ctx, PLL_STOPPED, VOLOOP_PLL_GetState(&invalid_sample.PLL));
}

static void test_disabled_state_ignores_power_measurements(VoloopTestContext* ctx) {
    PFC_HandleTypeDef pfc = make_started_pfc(1.0f, 0.0f, 0.01f, 0.0f);
    PFC_OutputTypeDef output = { 0 };
    PFC_InputTypeDef input = {
        .GridVoltage = 100.0f,
        .GridCurrent = NAN,
        .BusVoltage = NAN,
    };

    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_Stop(&pfc));
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_Sync(&pfc, &input, &output));
    TEST_EXPECT_STATE_EQ(ctx, PFC_DISABLED, VOLOOP_PFC_GetState(&pfc));
    TEST_EXPECT_FAULT_EQ(ctx, PFC_FAULT_NONE, VOLOOP_PFC_GetFaultCode(&pfc));
    TEST_EXPECT_STATE_EQ(ctx, PLL_RUNNING, VOLOOP_PLL_GetState(&pfc.PLL));
    expect_output_disabled(ctx, &output);
}

int main(void) {
    VoloopTestContext ctx = VOLOOP_TEST_CONTEXT_INIT;

    voloop_run_test(&ctx, "voltage loop generates current amplitude limit",
                    test_voltage_loop_generates_current_amplitude_limit);
    voloop_run_test(&ctx, "positive and negative bridge mapping",
                    test_positive_and_negative_bridge_mapping);
    voloop_run_test(&ctx, "negative half cycle aligns current measurement",
                    test_negative_half_cycle_aligns_current_measurement);
    voloop_run_test(&ctx, "zero crossing resets current controller",
                    test_zero_crossing_resets_current_controller);
    voloop_run_test(&ctx, "high frequency duty uses configured limits",
                    test_high_frequency_duty_uses_configured_limits);
    voloop_run_test(&ctx, "active protection latches fault and stops pll",
                    test_active_protection_latches_fault_and_stops_pll);
    voloop_run_test(&ctx, "disabled state ignores power measurements",
                    test_disabled_state_ignores_power_measurements);

    return voloop_test_report(&ctx);
}
