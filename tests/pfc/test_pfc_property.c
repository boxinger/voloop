#include "voloop_pfc.h"
#include "voloop_test.h"
#include "voloop_test_float.h"

#define TEST_TOLERANCE 1.0e-5f

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
        .triggerFrequency = 10000U,
        .initialFrequency = 50.0f,
        .initialRad = 0.0f,
    };

    return init;
}

static PLL_InitTypeDef make_pll_init(const PID_InitTypeDef* loop_pid, const NCO_InitTypeDef* nco) {
    PLL_InitTypeDef init = {
        .LoopFilterInit = loop_pid,
        .NCOInit = nco,
        .triggerFrequency = 10000.0f,
    };

    return init;
}

static PFC_InitTypeDef make_pfc_init(const PLL_InitTypeDef* pll, const PID_InitTypeDef* bus_pid,
                                     const PID_InitTypeDef* current_pid) {
    PFC_InitTypeDef init = {
        .PLLInit = pll,
        .BusVoltagePIDInit = bus_pid,
        .GridCurrentPIDInit = current_pid,
        .triggerFrequency = 10000.0f,
    };

    return init;
}

static void force_locked_phase(PFC_HandleTypeDef* pfc, float rad, float grid_peak) {
    pfc->State = PFC_RUNNING;
    pfc->PLL.State = PLL_STOPPED;
    pfc->PLL.LockState = PLL_LOCKED;
    pfc->PLL.PhaseQ31 = VOLOOP_DEF_RadToQ31(rad);
    pfc->GridVoltagePeak = grid_peak;
    pfc->GridVoltageDc = 0.0f;
    pfc->GridVoltageSquareAvg = (grid_peak * grid_peak) * 0.5f;
}

static PFC_HandleTypeDef make_started_locked_pfc(float rad, float grid_peak) {
    PFC_HandleTypeDef pfc = { 0 };
    PID_InitTypeDef pll_pid = make_pid_init(0.1f, 0.0f, 0.0f);
    PID_InitTypeDef bus_pid = make_pid_init(1.0f, 0.0f, 0.0f);
    PID_InitTypeDef current_pid = make_pid_init(1.0f, 0.0f, 0.0f);
    NCO_InitTypeDef nco = make_nco_init();
    PLL_InitTypeDef pll = make_pll_init(&pll_pid, &nco);
    PFC_InitTypeDef init = make_pfc_init(&pll, &bus_pid, &current_pid);

    (void)VOLOOP_PFC_Init(&pfc, &init);
    (void)VOLOOP_PFC_SetValue(&pfc, 380.0f, 10.0f);
    force_locked_phase(&pfc, rad, grid_peak);

    return pfc;
}

static void test_conductance_limit_uses_current_limit_state(VoloopTestContext* ctx) {
    PFC_HandleTypeDef pfc = make_started_locked_pfc(VOLOOP_Pi * 0.5f, 100.0f);
    PFC_OutputTypeDef output = { 0 };
    PFC_InputTypeDef input = {
        .GridVoltage = 100.0f,
        .GridCurrent = 0.0f,
        .BusVoltage = 200.0f,
    };

    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_Sync(&pfc, &input, &output));

    TEST_EXPECT_STATE_EQ(ctx, PFC_CURRENT_LIMIT, VOLOOP_PFC_GetState(&pfc));
    TEST_EXPECT_FAULT_EQ(ctx, PFC_NOERROR, VOLOOP_PFC_GetFaultCode(&pfc));
    TEST_EXPECT_FLOAT_NEAR(ctx, 10.0f, VOLOOP_PFC_GetCurrentReference(&pfc));
    TEST_EXPECT_STATE_EQ(ctx, VOLOOP_PWM_ENABLE, output.HighFrequencyPwmState);
}

static void test_ocp_enters_error_not_current_limit(VoloopTestContext* ctx) {
    PFC_HandleTypeDef pfc = make_started_locked_pfc(VOLOOP_Pi * 0.5f, 100.0f);
    PFC_OutputTypeDef output = { 0 };
    PFC_InputTypeDef input = {
        .GridVoltage = 100.0f,
        .GridCurrent = PFC_INPUT_OCTHRESHOLD + 0.5f,
        .BusVoltage = 200.0f,
    };

    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_ERROR, VOLOOP_PFC_Sync(&pfc, &input, &output));

    TEST_EXPECT_STATE_EQ(ctx, PFC_ERROR, VOLOOP_PFC_GetState(&pfc));
    TEST_EXPECT_FAULT_EQ(ctx, PFC_OCP, VOLOOP_PFC_GetFaultCode(&pfc));
    TEST_EXPECT_STATE_EQ(ctx, VOLOOP_PWM_DISABLED, output.HighFrequencyPwmState);
}

static void test_zero_crossing_freezes_current_loop_integral(VoloopTestContext* ctx) {
    PFC_HandleTypeDef pfc = make_started_locked_pfc(0.0f, 100.0f);
    PFC_OutputTypeDef output = { 0 };
    PFC_InputTypeDef input = {
        .GridVoltage = 0.0f,
        .GridCurrent = 0.0f,
        .BusVoltage = 300.0f,
    };

    pfc.GridCurrentPID.Integral = 3.0f;
    pfc.GridCurrentPID.PreviousError = -2.0f;
    pfc.Duty = 0.4f;

    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_Sync(&pfc, &input, &output));

    TEST_EXPECT_STATE_EQ(ctx, VOLOOP_PWM_DISABLED, output.HighFrequencyPwmState);
    TEST_EXPECT_STATE_EQ(ctx, PFC_LINE_ZERO, output.LinePolarity);
    TEST_EXPECT_FLOAT_NEAR(ctx, 3.0f, pfc.GridCurrentPID.Integral);
    TEST_EXPECT_FLOAT_NEAR(ctx, -2.0f, pfc.GridCurrentPID.PreviousError);
    TEST_EXPECT_FLOAT_NEAR(ctx, 0.4f, VOLOOP_PFC_GetDuty(&pfc));
}

static void test_current_reference_sign_follows_pll_half_cycle(VoloopTestContext* ctx) {
    PFC_HandleTypeDef positive = make_started_locked_pfc(VOLOOP_Pi * 0.5f, 100.0f);
    PFC_HandleTypeDef negative = make_started_locked_pfc(-VOLOOP_Pi * 0.5f, 100.0f);
    PFC_OutputTypeDef output = { 0 };
    PFC_InputTypeDef input = {
        .GridVoltage = 100.0f,
        .GridCurrent = 0.0f,
        .BusVoltage = 379.0f,
    };

    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_Sync(&positive, &input, &output));
    VOLOOP_EXPECT_TRUE(ctx, VOLOOP_PFC_GetCurrentReference(&positive) > 0.0f);
    TEST_EXPECT_STATE_EQ(ctx, PFC_LINE_POSITIVE, VOLOOP_PFC_GetLinePolarity(&positive));

    input.GridVoltage = -100.0f;
    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_Sync(&negative, &input, &output));
    VOLOOP_EXPECT_TRUE(ctx, VOLOOP_PFC_GetCurrentReference(&negative) < 0.0f);
    TEST_EXPECT_STATE_EQ(ctx, PFC_LINE_NEGATIVE, VOLOOP_PFC_GetLinePolarity(&negative));
}

static void test_current_loop_duty_is_limited(VoloopTestContext* ctx) {
    PFC_HandleTypeDef pfc = make_started_locked_pfc(VOLOOP_Pi * 0.5f, 100.0f);
    PFC_OutputTypeDef output = { 0 };
    PFC_InputTypeDef input = {
        .GridVoltage = 100.0f,
        .GridCurrent = -20.0f,
        .BusVoltage = 200.0f,
    };

    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_Sync(&pfc, &input, &output));

    TEST_EXPECT_STATE_EQ(ctx, VOLOOP_PWM_ENABLE, output.HighFrequencyPwmState);
    TEST_EXPECT_FLOAT_NEAR(ctx, PFC_MAX_DUTY, output.HighFrequencyDuty);
    TEST_EXPECT_FLOAT_NEAR(ctx, PFC_MAX_DUTY, VOLOOP_PFC_GetDuty(&pfc));
}

int main(void) {
    VoloopTestContext ctx = VOLOOP_TEST_CONTEXT_INIT;

    voloop_run_test(&ctx, "conductance limit uses current limit state",
                    test_conductance_limit_uses_current_limit_state);
    voloop_run_test(&ctx, "ocp enters error not current limit",
                    test_ocp_enters_error_not_current_limit);
    voloop_run_test(&ctx, "zero crossing freezes current loop integral",
                    test_zero_crossing_freezes_current_loop_integral);
    voloop_run_test(&ctx, "current reference sign follows pll half cycle",
                    test_current_reference_sign_follows_pll_half_cycle);
    voloop_run_test(&ctx, "current loop duty is limited", test_current_loop_duty_is_limited);

    return voloop_test_report(&ctx);
}
