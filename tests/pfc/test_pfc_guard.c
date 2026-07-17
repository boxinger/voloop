#include "voloop_pfc.h"
#include "voloop_test.h"
#include "voloop_test_float.h"

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

static void test_init_rejects_null_and_missing_children(VoloopTestContext* ctx) {
    PFC_HandleTypeDef pfc = { 0 };
    PID_InitTypeDef pll_pid = make_pid_init(0.1f, 0.0f, 0.0f);
    PID_InitTypeDef bus_pid = make_pid_init(1.0f, 0.0f, 0.0f);
    PID_InitTypeDef current_pid = make_pid_init(0.5f, 0.0f, 0.0f);
    NCO_InitTypeDef nco = make_nco_init();
    PLL_InitTypeDef pll = make_pll_init(&pll_pid, &nco);
    PFC_InitTypeDef init = make_pfc_init(&pll, &bus_pid, &current_pid);

    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PFC_Init(NULL, &init));
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PFC_Init(&pfc, NULL));

    init.PLLInit = NULL;
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PFC_Init(&pfc, &init));

    init = make_pfc_init(&pll, NULL, &current_pid);
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PFC_Init(&pfc, &init));

    init = make_pfc_init(&pll, &bus_pid, NULL);
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PFC_Init(&pfc, &init));

    init = make_pfc_init(&pll, &bus_pid, &current_pid);
    init.triggerFrequency = 0.0f;
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PFC_Init(&pfc, &init));
}

static void test_init_start_stop_lifecycle(VoloopTestContext* ctx) {
    PFC_HandleTypeDef pfc = { 0 };
    PID_InitTypeDef pll_pid = make_pid_init(0.1f, 0.0f, 0.0f);
    PID_InitTypeDef bus_pid = make_pid_init(1.0f, 0.0f, 0.0f);
    PID_InitTypeDef current_pid = make_pid_init(0.5f, 0.0f, 0.0f);
    NCO_InitTypeDef nco = make_nco_init();
    PLL_InitTypeDef pll = make_pll_init(&pll_pid, &nco);
    PFC_InitTypeDef init = make_pfc_init(&pll, &bus_pid, &current_pid);

    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_Init(&pfc, &init));
    TEST_EXPECT_STATE_EQ(ctx, PFC_DISABLED, VOLOOP_PFC_GetState(&pfc));
    TEST_EXPECT_FAULT_EQ(ctx, PFC_NOERROR, VOLOOP_PFC_GetFaultCode(&pfc));

    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_Start(&pfc));
    TEST_EXPECT_STATE_EQ(ctx, PFC_WAIT_PLL, VOLOOP_PFC_GetState(&pfc));

    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_Stop(&pfc));
    TEST_EXPECT_STATE_EQ(ctx, PFC_DISABLED, VOLOOP_PFC_GetState(&pfc));
}

static void test_set_value_rejects_invalid_limits(VoloopTestContext* ctx) {
    PFC_HandleTypeDef pfc = { 0 };
    PID_InitTypeDef pll_pid = make_pid_init(0.1f, 0.0f, 0.0f);
    PID_InitTypeDef bus_pid = make_pid_init(1.0f, 0.0f, 0.0f);
    PID_InitTypeDef current_pid = make_pid_init(0.5f, 0.0f, 0.0f);
    NCO_InitTypeDef nco = make_nco_init();
    PLL_InitTypeDef pll = make_pll_init(&pll_pid, &nco);
    PFC_InitTypeDef init = make_pfc_init(&pll, &bus_pid, &current_pid);

    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_Init(&pfc, &init));
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PFC_SetValue(NULL, 380.0f, 10.0f));
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PFC_SetValue(&pfc, 0.0f, 10.0f));
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM,
                          VOLOOP_PFC_SetValue(&pfc, PFC_BUS_OVTHRESHOLD + 1.0f, 10.0f));
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PFC_SetValue(&pfc, 380.0f, 0.0f));
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM,
                          VOLOOP_PFC_SetValue(&pfc, 380.0f, PFC_INPUT_OCTHRESHOLD + 1.0f));

    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_SetValue(&pfc, 380.0f, 10.0f));
}

static void test_disabled_sync_writes_safe_output(VoloopTestContext* ctx) {
    PFC_HandleTypeDef pfc = { 0 };
    PFC_OutputTypeDef output = { 0 };
    PFC_InputTypeDef input = {
        .GridVoltage = 0.0f,
        .GridCurrent = 0.0f,
        .BusVoltage = 0.0f,
    };
    PID_InitTypeDef pll_pid = make_pid_init(0.1f, 0.0f, 0.0f);
    PID_InitTypeDef bus_pid = make_pid_init(1.0f, 0.0f, 0.0f);
    PID_InitTypeDef current_pid = make_pid_init(0.5f, 0.0f, 0.0f);
    NCO_InitTypeDef nco = make_nco_init();
    PLL_InitTypeDef pll = make_pll_init(&pll_pid, &nco);
    PFC_InitTypeDef init = make_pfc_init(&pll, &bus_pid, &current_pid);

    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_Init(&pfc, &init));
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_Sync(&pfc, &input, &output));
    TEST_EXPECT_STATE_EQ(ctx, VOLOOP_PWM_DISABLED, output.HighFrequencyPwmState);
    TEST_EXPECT_FLOAT_EQ(ctx, 0.0f, output.HighFrequencyDuty);
    TEST_EXPECT_STATE_EQ(ctx, PFC_LINE_ZERO, output.LinePolarity);
}

static void test_protection_faults_latch_and_clear(VoloopTestContext* ctx) {
    PFC_HandleTypeDef pfc = { 0 };
    PFC_OutputTypeDef output = { 0 };
    PID_InitTypeDef pll_pid = make_pid_init(0.1f, 0.0f, 0.0f);
    PID_InitTypeDef bus_pid = make_pid_init(1.0f, 0.0f, 0.0f);
    PID_InitTypeDef current_pid = make_pid_init(0.5f, 0.0f, 0.0f);
    NCO_InitTypeDef nco = make_nco_init();
    PLL_InitTypeDef pll = make_pll_init(&pll_pid, &nco);
    PFC_InitTypeDef init = make_pfc_init(&pll, &bus_pid, &current_pid);

    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_Init(&pfc, &init));
    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_Start(&pfc));

    PFC_InputTypeDef ocp_input = {
        .GridVoltage = 100.0f,
        .GridCurrent = PFC_INPUT_OCTHRESHOLD + 1.0f,
        .BusVoltage = 300.0f,
    };
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_ERROR, VOLOOP_PFC_Sync(&pfc, &ocp_input, &output));
    TEST_EXPECT_STATE_EQ(ctx, PFC_ERROR, VOLOOP_PFC_GetState(&pfc));
    TEST_EXPECT_FAULT_EQ(ctx, PFC_OCP, VOLOOP_PFC_GetFaultCode(&pfc));

    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_ClearFaultCode(&pfc));
    TEST_EXPECT_STATE_EQ(ctx, PFC_DISABLED, VOLOOP_PFC_GetState(&pfc));
    TEST_EXPECT_FAULT_EQ(ctx, PFC_NOERROR, VOLOOP_PFC_GetFaultCode(&pfc));

    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PFC_Start(&pfc));
    PFC_InputTypeDef ovp_input = {
        .GridVoltage = 100.0f,
        .GridCurrent = 0.0f,
        .BusVoltage = PFC_BUS_OVTHRESHOLD + 1.0f,
    };
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_ERROR, VOLOOP_PFC_Sync(&pfc, &ovp_input, &output));
    TEST_EXPECT_STATE_EQ(ctx, PFC_ERROR, VOLOOP_PFC_GetState(&pfc));
    TEST_EXPECT_FAULT_EQ(ctx, PFC_OVP, VOLOOP_PFC_GetFaultCode(&pfc));
}

static void test_getters_return_safe_defaults_for_null(VoloopTestContext* ctx) {
    TEST_EXPECT_STATE_EQ(ctx, PFC_ERROR, VOLOOP_PFC_GetState(NULL));
    TEST_EXPECT_FAULT_EQ(ctx, PFC_INVALID, VOLOOP_PFC_GetFaultCode(NULL));
    TEST_EXPECT_STATE_EQ(ctx, PLL_UNLOCKED, VOLOOP_PFC_GetPLLLockState(NULL));
    TEST_EXPECT_FLOAT_EQ(ctx, 0.0f, VOLOOP_PFC_GetDuty(NULL));
    TEST_EXPECT_FLOAT_EQ(ctx, 0.0f, VOLOOP_PFC_GetCurrentReference(NULL));
    TEST_EXPECT_FLOAT_EQ(ctx, 0.0f, VOLOOP_PFC_GetGridVoltagePeak(NULL));
    TEST_EXPECT_STATE_EQ(ctx, PFC_LINE_ZERO, VOLOOP_PFC_GetLinePolarity(NULL));
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PFC_DeInit(NULL));
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PFC_Start(NULL));
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PFC_Stop(NULL));
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PFC_ClearFaultCode(NULL));
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PFC_Sync(NULL, NULL, NULL));
}

int main(void) {
    VoloopTestContext ctx = VOLOOP_TEST_CONTEXT_INIT;

    voloop_run_test(&ctx, "init rejects invalid arguments",
                    test_init_rejects_null_and_missing_children);
    voloop_run_test(&ctx, "init start stop lifecycle", test_init_start_stop_lifecycle);
    voloop_run_test(&ctx, "set value rejects invalid limits",
                    test_set_value_rejects_invalid_limits);
    voloop_run_test(&ctx, "disabled sync writes safe output",
                    test_disabled_sync_writes_safe_output);
    voloop_run_test(&ctx, "protection faults latch and clear",
                    test_protection_faults_latch_and_clear);
    voloop_run_test(&ctx, "getters return safe defaults for null",
                    test_getters_return_safe_defaults_for_null);

    return voloop_test_report(&ctx);
}
