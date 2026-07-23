#include "voloop_pfc.h"
#include "voloop_test.h"
#include "voloop_test_float.h"

#include <math.h>

#define TEST_EXPECT_STATUS_EQ(ctx, expected, actual)                                               \
    VOLOOP_EXPECT_EQ_INT((ctx), (expected), (actual))
#define TEST_REQUIRE_STATUS_EQ(ctx, expected, actual)                                              \
    VOLOOP_REQUIRE_EQ_INT((ctx), (expected), (actual))
#define TEST_EXPECT_STATE_EQ(ctx, expected, actual)                                                \
    VOLOOP_EXPECT_EQ_INT((ctx), (expected), (actual))
#define TEST_EXPECT_FAULT_EQ(ctx, expected, actual)                                                \
    VOLOOP_EXPECT_EQ_INT((ctx), (expected), (actual))
#define TEST_EXPECT_FLOAT_EQ(ctx, expected, actual)                                                \
    VOLOOP_EXPECT_FLOAT_NEAR((ctx), (expected), (actual), 0.0f)

#define TEST_TRIGGER_FREQUENCY 10000.0f

static PID_InitTypeDef make_pid_init(float kp, float ki, float kd) {
    PID_InitTypeDef init = { 0 };

    init.mode = PID_Discrete;
    init.init.Discrete.KpDiscrete = kp;
    init.init.Discrete.KiDiscrete = ki;
    init.init.Discrete.KdDiscrete = kd;

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

static PLL_InitTypeDef make_pll_init(const PID_InitTypeDef* loop_pid, const NCO_InitTypeDef* nco,
                                     float trigger_frequency) {
    PLL_InitTypeDef init = {
        .LoopFilterInit = loop_pid,
        .NCOInit = nco,
        .triggerFrequency = trigger_frequency,
    };

    return init;
}

static PFC_ConfigTypeDef make_pfc_config(void) {
    PFC_ConfigTypeDef config = {
        .BusOverVoltageThreshold = 450.0f,
        .GridOverCurrentThreshold = 30.0f,
        .MinHighFrequencyDuty = 0.02f,
        .MaxHighFrequencyDuty = 0.98f,
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

static void test_init_rejects_invalid_arguments(VoloopTestContext* ctx) {
    PFC_HandleTypeDef pfc = { 0 };
    PID_InitTypeDef pll_pid = make_pid_init(0.1f, 0.0f, 0.0f);
    PID_InitTypeDef bus_pid = make_pid_init(1.0f, 0.01f, 0.0f);
    PID_InitTypeDef current_pid = make_pid_init(0.5f, 0.02f, 0.0f);
    NCO_InitTypeDef nco = make_nco_init();
    PLL_InitTypeDef pll = make_pll_init(&pll_pid, &nco, TEST_TRIGGER_FREQUENCY);
    PFC_ConfigTypeDef config = make_pfc_config();
    PFC_InitTypeDef init = make_pfc_init(&pll, &bus_pid, &current_pid, &config);

    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PFC_Init(NULL, &init));
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PFC_Init(&pfc, NULL));

    init.PLLInit = NULL;
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PFC_Init(&pfc, &init));
    init = make_pfc_init(&pll, NULL, &current_pid, &config);
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PFC_Init(&pfc, &init));
    init = make_pfc_init(&pll, &bus_pid, NULL, &config);
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PFC_Init(&pfc, &init));
    init = make_pfc_init(&pll, &bus_pid, &current_pid, NULL);
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PFC_Init(&pfc, &init));

    init = make_pfc_init(&pll, &bus_pid, &current_pid, &config);
    init.TriggerFrequency = 0.0f;
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PFC_Init(&pfc, &init));

    PLL_InitTypeDef mismatched_pll = make_pll_init(&pll_pid, &nco, TEST_TRIGGER_FREQUENCY - 1.0f);
    init = make_pfc_init(&mismatched_pll, &bus_pid, &current_pid, &config);
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PFC_Init(&pfc, &init));

    bus_pid.mode = (PID_InitModeTypeDef)99;
    init = make_pfc_init(&pll, &bus_pid, &current_pid, &config);
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PFC_Init(&pfc, &init));
    TEST_EXPECT_STATE_EQ(ctx, PFC_RESET, VOLOOP_PFC_GetState(&pfc));
}

static void test_init_rejects_invalid_config(VoloopTestContext* ctx) {
    PFC_HandleTypeDef pfc = { 0 };
    PID_InitTypeDef pll_pid = make_pid_init(0.1f, 0.0f, 0.0f);
    PID_InitTypeDef bus_pid = make_pid_init(1.0f, 0.01f, 0.0f);
    PID_InitTypeDef current_pid = make_pid_init(0.5f, 0.02f, 0.0f);
    NCO_InitTypeDef nco = make_nco_init();
    PLL_InitTypeDef pll = make_pll_init(&pll_pid, &nco, TEST_TRIGGER_FREQUENCY);
    PFC_ConfigTypeDef config = make_pfc_config();
    PFC_InitTypeDef init = make_pfc_init(&pll, &bus_pid, &current_pid, &config);

    config.BusOverVoltageThreshold = 0.0f;
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PFC_Init(&pfc, &init));

    config = make_pfc_config();
    config.GridOverCurrentThreshold = NAN;
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PFC_Init(&pfc, &init));

    config = make_pfc_config();
    config.MinHighFrequencyDuty = 0.9f;
    config.MaxHighFrequencyDuty = 0.8f;
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PFC_Init(&pfc, &init));

    config = make_pfc_config();
    config.MaxHighFrequencyDuty = 1.0f;
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PFC_Init(&pfc, &init));

    config = make_pfc_config();
    config.ZeroCrossingDeadband = 1.0f;
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PFC_Init(&pfc, &init));
}

static void test_lifecycle_keeps_pll_running(VoloopTestContext* ctx) {
    PFC_HandleTypeDef pfc = { 0 };
    PID_InitTypeDef pll_pid = make_pid_init(0.1f, 0.0f, 0.0f);
    PID_InitTypeDef bus_pid = make_pid_init(1.0f, 0.01f, 0.0f);
    PID_InitTypeDef current_pid = make_pid_init(0.5f, 0.02f, 0.0f);
    NCO_InitTypeDef nco = make_nco_init();
    PLL_InitTypeDef pll = make_pll_init(&pll_pid, &nco, TEST_TRIGGER_FREQUENCY);
    PFC_ConfigTypeDef config = make_pfc_config();
    PFC_InitTypeDef init = make_pfc_init(&pll, &bus_pid, &current_pid, &config);

    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_Init(&pfc, &init));
    TEST_EXPECT_STATE_EQ(ctx, PFC_DISABLED, VOLOOP_PFC_GetState(&pfc));
    TEST_EXPECT_FAULT_EQ(ctx, PFC_FAULT_NONE, VOLOOP_PFC_GetFaultCode(&pfc));
    TEST_EXPECT_STATE_EQ(ctx, PLL_RUNNING, VOLOOP_PLL_GetState(&pfc.PLL));
    VOLOOP_EXPECT_EQ_INT(ctx, 0, pfc.ReferenceConfigured);

    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_STATE, VOLOOP_PFC_Start(&pfc));
    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_SetValue(&pfc, 400.0f, 20.0f));

    pfc.BusVoltagePID.Integral = 3.0f;
    pfc.GridCurrentPID.Integral = 4.0f;
    pfc.Modulation = 0.5f;
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_Start(&pfc));
    TEST_EXPECT_STATE_EQ(ctx, PFC_WAIT_PLL, VOLOOP_PFC_GetState(&pfc));
    TEST_EXPECT_STATE_EQ(ctx, PLL_RUNNING, VOLOOP_PLL_GetState(&pfc.PLL));
    TEST_EXPECT_FLOAT_EQ(ctx, 0.0f, pfc.BusVoltagePID.Integral);
    TEST_EXPECT_FLOAT_EQ(ctx, 0.0f, pfc.GridCurrentPID.Integral);
    TEST_EXPECT_FLOAT_EQ(ctx, 0.0f, VOLOOP_PFC_GetModulation(&pfc));
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_Start(&pfc));

    pfc.GridCurrentPID.PreviousError = 2.0f;
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_Stop(&pfc));
    TEST_EXPECT_STATE_EQ(ctx, PFC_DISABLED, VOLOOP_PFC_GetState(&pfc));
    TEST_EXPECT_STATE_EQ(ctx, PLL_RUNNING, VOLOOP_PLL_GetState(&pfc.PLL));
    TEST_EXPECT_FLOAT_EQ(ctx, 0.0f, pfc.GridCurrentPID.PreviousError);
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_Stop(&pfc));

    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_DeInit(&pfc));
    TEST_EXPECT_STATE_EQ(ctx, PFC_RESET, VOLOOP_PFC_GetState(&pfc));
    TEST_EXPECT_STATE_EQ(ctx, PLL_RESET, VOLOOP_PLL_GetState(&pfc.PLL));
}

static void test_set_value_validates_limits_and_state(VoloopTestContext* ctx) {
    PFC_HandleTypeDef pfc = { 0 };
    PID_InitTypeDef pll_pid = make_pid_init(0.1f, 0.0f, 0.0f);
    PID_InitTypeDef bus_pid = make_pid_init(1.0f, 0.01f, 0.0f);
    PID_InitTypeDef current_pid = make_pid_init(0.5f, 0.02f, 0.0f);
    NCO_InitTypeDef nco = make_nco_init();
    PLL_InitTypeDef pll = make_pll_init(&pll_pid, &nco, TEST_TRIGGER_FREQUENCY);
    PFC_ConfigTypeDef config = make_pfc_config();
    PFC_InitTypeDef init = make_pfc_init(&pll, &bus_pid, &current_pid, &config);

    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PFC_SetValue(NULL, 400.0f, 20.0f));
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_STATE, VOLOOP_PFC_SetValue(&pfc, 400.0f, 20.0f));

    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_Init(&pfc, &init));
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PFC_SetValue(&pfc, NAN, 20.0f));
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PFC_SetValue(&pfc, 0.0f, 20.0f));
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM,
                          VOLOOP_PFC_SetValue(&pfc, config.BusOverVoltageThreshold, 20.0f));
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PFC_SetValue(&pfc, 400.0f, 0.0f));
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM,
                          VOLOOP_PFC_SetValue(&pfc, 400.0f, config.GridOverCurrentThreshold));

    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_SetValue(&pfc, 400.0f, 20.0f));
    TEST_EXPECT_FLOAT_EQ(ctx, 400.0f, pfc.TargetBusVoltage);
    TEST_EXPECT_FLOAT_EQ(ctx, 20.0f, pfc.MaxGridCurrent);
    VOLOOP_EXPECT_EQ_INT(ctx, 1, pfc.ReferenceConfigured);

    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_Start(&pfc));
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_SetValue(&pfc, 410.0f, 21.0f));
    TEST_EXPECT_FLOAT_EQ(ctx, 410.0f, pfc.TargetBusVoltage);
    TEST_EXPECT_FLOAT_EQ(ctx, 21.0f, pfc.MaxGridCurrent);

    pfc.State = PFC_ERROR;
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_STATE, VOLOOP_PFC_SetValue(&pfc, 400.0f, 20.0f));
}

static void test_clear_fault_restarts_pll(VoloopTestContext* ctx) {
    PFC_HandleTypeDef pfc = { 0 };
    PID_InitTypeDef pll_pid = make_pid_init(0.1f, 0.0f, 0.0f);
    PID_InitTypeDef bus_pid = make_pid_init(1.0f, 0.01f, 0.0f);
    PID_InitTypeDef current_pid = make_pid_init(0.5f, 0.02f, 0.0f);
    NCO_InitTypeDef nco = make_nco_init();
    PLL_InitTypeDef pll = make_pll_init(&pll_pid, &nco, TEST_TRIGGER_FREQUENCY);
    PFC_ConfigTypeDef config = make_pfc_config();
    PFC_InitTypeDef init = make_pfc_init(&pll, &bus_pid, &current_pid, &config);

    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_Init(&pfc, &init));
    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_SetValue(&pfc, 400.0f, 20.0f));
    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PLL_Stop(&pfc.PLL));
    pfc.State = PFC_ERROR;
    pfc.FaultCode = PFC_FAULT_INPUT_OVERCURRENT;
    pfc.BusVoltagePID.Integral = 2.0f;
    pfc.GridCurrentPID.Integral = 3.0f;

    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_ClearFaultCode(&pfc));
    TEST_EXPECT_STATE_EQ(ctx, PFC_DISABLED, VOLOOP_PFC_GetState(&pfc));
    TEST_EXPECT_FAULT_EQ(ctx, PFC_FAULT_NONE, VOLOOP_PFC_GetFaultCode(&pfc));
    TEST_EXPECT_STATE_EQ(ctx, PLL_RUNNING, VOLOOP_PLL_GetState(&pfc.PLL));
    TEST_EXPECT_STATE_EQ(ctx, PLL_UNLOCKED, VOLOOP_PFC_GetPLLLockState(&pfc));
    TEST_EXPECT_FLOAT_EQ(ctx, 0.0f, pfc.BusVoltagePID.Integral);
    TEST_EXPECT_FLOAT_EQ(ctx, 0.0f, pfc.GridCurrentPID.Integral);
    VOLOOP_EXPECT_EQ_INT(ctx, 1, pfc.ReferenceConfigured);

    pfc.State = PFC_ERROR;
    pfc.FaultCode = PFC_FAULT_PLL;
    pfc.PLL.State = PLL_ERROR;
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_STATE, VOLOOP_PFC_ClearFaultCode(&pfc));
    TEST_EXPECT_STATE_EQ(ctx, PFC_ERROR, VOLOOP_PFC_GetState(&pfc));
}

static void test_getters_and_invalid_sync_are_safe(VoloopTestContext* ctx) {
    PFC_HandleTypeDef pfc = { 0 };
    PFC_InputTypeDef input = {
        .GridVoltage = 100.0f,
        .GridCurrent = 2.0f,
        .BusVoltage = 300.0f,
    };
    PFC_OutputTypeDef output = {
        .LeftLegPwmState = VOLOOP_PWM_ENABLE,
        .LeftLegDuty = 0.5f,
        .RightLegPwmState = VOLOOP_PWM_ENABLE,
        .RightLegDuty = 0.5f,
    };

    TEST_EXPECT_STATE_EQ(ctx, PFC_ERROR, VOLOOP_PFC_GetState(NULL));
    TEST_EXPECT_FAULT_EQ(ctx, PFC_FAULT_INVALID, VOLOOP_PFC_GetFaultCode(NULL));
    TEST_EXPECT_STATE_EQ(ctx, PLL_UNLOCKED, VOLOOP_PFC_GetPLLLockState(NULL));
    TEST_EXPECT_FLOAT_EQ(ctx, 0.0f, VOLOOP_PFC_GetModulation(NULL));
    TEST_EXPECT_FLOAT_EQ(ctx, 0.0f, VOLOOP_PFC_GetCurrentAmplitudeReference(NULL));
    TEST_EXPECT_FLOAT_EQ(ctx, 0.0f, VOLOOP_PFC_GetCurrentReference(NULL));
    TEST_EXPECT_STATE_EQ(ctx, PFC_LINE_ZERO, VOLOOP_PFC_GetLinePolarity(NULL));
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PFC_DeInit(NULL));
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PFC_Start(NULL));
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PFC_Stop(NULL));
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PFC_ClearFaultCode(NULL));

    pfc.CurrentAmplitudeReference = 12.0f;
    pfc.CurrentReference = -6.0f;
    pfc.Modulation = -0.25f;
    pfc.LinePolarity = PFC_LINE_NEGATIVE;
    TEST_EXPECT_FLOAT_EQ(ctx, 12.0f, VOLOOP_PFC_GetCurrentAmplitudeReference(&pfc));
    TEST_EXPECT_FLOAT_EQ(ctx, -6.0f, VOLOOP_PFC_GetCurrentReference(&pfc));
    TEST_EXPECT_FLOAT_EQ(ctx, -0.25f, VOLOOP_PFC_GetModulation(&pfc));
    TEST_EXPECT_STATE_EQ(ctx, PFC_LINE_NEGATIVE, VOLOOP_PFC_GetLinePolarity(&pfc));

    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PFC_Sync(NULL, &input, &output));
    TEST_EXPECT_STATE_EQ(ctx, VOLOOP_PWM_DISABLED, output.LeftLegPwmState);
    TEST_EXPECT_FLOAT_EQ(ctx, 0.0f, output.LeftLegDuty);
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PFC_Sync(&pfc, NULL, &output));
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PFC_Sync(&pfc, &input, NULL));
}

static void test_sync_advances_pll_and_reports_readiness(VoloopTestContext* ctx) {
    PFC_HandleTypeDef pfc = { 0 };
    PID_InitTypeDef pll_pid = make_pid_init(0.1f, 0.0f, 0.0f);
    PID_InitTypeDef bus_pid = make_pid_init(1.0f, 0.01f, 0.0f);
    PID_InitTypeDef current_pid = make_pid_init(0.5f, 0.02f, 0.0f);
    NCO_InitTypeDef nco = make_nco_init();
    PLL_InitTypeDef pll = make_pll_init(&pll_pid, &nco, TEST_TRIGGER_FREQUENCY);
    PFC_ConfigTypeDef config = make_pfc_config();
    PFC_InitTypeDef init = make_pfc_init(&pll, &bus_pid, &current_pid, &config);
    PFC_InputTypeDef input = {
        .GridVoltage = 100.0f,
        .GridCurrent = NAN,
        .BusVoltage = NAN,
    };
    PFC_OutputTypeDef output = {
        .LeftLegPwmState = VOLOOP_PWM_ENABLE,
        .LeftLegDuty = 0.5f,
        .RightLegPwmState = VOLOOP_PWM_ENABLE,
        .RightLegDuty = 0.5f,
    };

    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_Init(&pfc, &init));
    int32_t phase_before = VOLOOP_PLL_GetPhaseQ31(&pfc.PLL);
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_Sync(&pfc, &input, &output));
    VOLOOP_EXPECT_TRUE(ctx, VOLOOP_PLL_GetPhaseQ31(&pfc.PLL) != phase_before);
    TEST_EXPECT_STATE_EQ(ctx, PFC_DISABLED, VOLOOP_PFC_GetState(&pfc));
    TEST_EXPECT_STATE_EQ(ctx, VOLOOP_PWM_DISABLED, output.LeftLegPwmState);
    TEST_EXPECT_FLOAT_EQ(ctx, 0.0f, output.LeftLegDuty);
    TEST_EXPECT_STATE_EQ(ctx, VOLOOP_PWM_DISABLED, output.RightLegPwmState);
    TEST_EXPECT_FLOAT_EQ(ctx, 0.0f, output.RightLegDuty);

    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_SetValue(&pfc, 400.0f, 20.0f));
    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_Start(&pfc));
    input.GridCurrent = 0.0f;
    input.BusVoltage = 390.0f;
    pfc.PLL.LockState = PLL_UNLOCKED;
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_Sync(&pfc, &input, &output));
    TEST_EXPECT_STATE_EQ(ctx, PFC_WAIT_PLL, VOLOOP_PFC_GetState(&pfc));

    pfc.PLL.LockState = PLL_LOCKED;
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_Sync(&pfc, &input, &output));
    TEST_EXPECT_STATE_EQ(ctx, PFC_RUNNING, VOLOOP_PFC_GetState(&pfc));
    TEST_EXPECT_STATE_EQ(ctx, VOLOOP_PWM_ENABLE, output.LeftLegPwmState);
    TEST_EXPECT_STATE_EQ(ctx, VOLOOP_PWM_ENABLE, output.RightLegPwmState);

    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_Stop(&pfc));
    input.GridVoltage = NAN;
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_ERROR, VOLOOP_PFC_Sync(&pfc, &input, &output));
    TEST_EXPECT_STATE_EQ(ctx, PFC_ERROR, VOLOOP_PFC_GetState(&pfc));
    TEST_EXPECT_FAULT_EQ(ctx, PFC_FAULT_INVALID_SAMPLE, VOLOOP_PFC_GetFaultCode(&pfc));
    TEST_EXPECT_STATE_EQ(ctx, PLL_STOPPED, VOLOOP_PLL_GetState(&pfc.PLL));
    TEST_EXPECT_STATE_EQ(ctx, VOLOOP_PWM_DISABLED, output.LeftLegPwmState);
    TEST_EXPECT_STATE_EQ(ctx, VOLOOP_PWM_DISABLED, output.RightLegPwmState);

    input.GridVoltage = 100.0f;
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_STATE, VOLOOP_PFC_Sync(&pfc, &input, &output));

    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_ClearFaultCode(&pfc));
    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PLL_Stop(&pfc.PLL));
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_Sync(&pfc, &input, &output));
    TEST_EXPECT_STATE_EQ(ctx, PFC_DISABLED, VOLOOP_PFC_GetState(&pfc));
    TEST_EXPECT_FAULT_EQ(ctx, PFC_FAULT_NONE, VOLOOP_PFC_GetFaultCode(&pfc));
    TEST_EXPECT_STATE_EQ(ctx, PLL_RUNNING, VOLOOP_PLL_GetState(&pfc.PLL));

    pfc.PLL.State = PLL_RESET;
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_STATE, VOLOOP_PFC_Sync(&pfc, &input, &output));
    TEST_EXPECT_STATE_EQ(ctx, PFC_DISABLED, VOLOOP_PFC_GetState(&pfc));
    TEST_EXPECT_FAULT_EQ(ctx, PFC_FAULT_NONE, VOLOOP_PFC_GetFaultCode(&pfc));
}

int main(void) {
    VoloopTestContext ctx = VOLOOP_TEST_CONTEXT_INIT;

    voloop_run_test(&ctx, "init rejects invalid arguments", test_init_rejects_invalid_arguments);
    voloop_run_test(&ctx, "init rejects invalid config", test_init_rejects_invalid_config);
    voloop_run_test(&ctx, "lifecycle keeps pll running", test_lifecycle_keeps_pll_running);
    voloop_run_test(&ctx, "set value validates limits and state",
                    test_set_value_validates_limits_and_state);
    voloop_run_test(&ctx, "clear fault restarts pll", test_clear_fault_restarts_pll);
    voloop_run_test(&ctx, "getters and invalid sync are safe",
                    test_getters_and_invalid_sync_are_safe);
    voloop_run_test(&ctx, "sync advances pll and reports readiness",
                    test_sync_advances_pll_and_reports_readiness);

    return voloop_test_report(&ctx);
}
