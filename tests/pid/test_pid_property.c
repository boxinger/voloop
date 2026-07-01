#include "voloop_pid.h"
#include "voloop_test.h"
#include "voloop_test_float.h"

#define TEST_TOLERANCE 1.0e-5f

#define TEST_EXPECT_STATUS_EQ(ctx, expected, actual) VOLOOP_EXPECT_EQ_INT((ctx), (expected), (actual))
#define TEST_REQUIRE_STATUS_EQ(ctx, expected, actual) VOLOOP_REQUIRE_EQ_INT((ctx), (expected), (actual))
#define TEST_EXPECT_STATE_EQ(ctx, expected, actual) VOLOOP_EXPECT_EQ_INT((ctx), (expected), (actual))
#define TEST_EXPECT_FLOAT_NEAR(ctx, expected, actual) \
    VOLOOP_EXPECT_FLOAT_NEAR((ctx), (expected), (actual), TEST_TOLERANCE)

static PID_InitTypeDef make_discrete_init(float kp, float ki, float kd) {
    PID_InitTypeDef init = {0};

    init.mode = PID_Discrete;
    init.init.Discrete.KpDiscrete = kp;
    init.init.Discrete.KiDiscrete = ki;
    init.init.Discrete.KdDiscrete = kd;

    return init;
}

/* Verify that discrete PID initialization copies Kp, Ki, and Kd into the handle. */
static void test_discrete_init_copies_gains(VoloopTestContext* ctx) {
    PID_HandleTypeDef pid = {0};
    PID_InitTypeDef init = make_discrete_init(1.25f, 0.5f, 0.125f);

    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PID_Init(&pid, &init));
    TEST_EXPECT_FLOAT_NEAR(ctx, 1.25f, pid.KpDiscrete);
    TEST_EXPECT_FLOAT_NEAR(ctx, 0.5f, pid.KiDiscrete);
    TEST_EXPECT_FLOAT_NEAR(ctx, 0.125f, pid.KdDiscrete);
}

/* Verify the plain compute contract for error, integral, derivative, output, and history. */
static void test_compute_updates_state_by_difference_equation(VoloopTestContext* ctx) {
    PID_HandleTypeDef pid = {0};
    PID_InitTypeDef init = make_discrete_init(2.0f, 0.5f, 0.25f);

    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PID_Init(&pid, &init));
    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PID_SetIntegral(&pid, 1.0f));
    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PID_SetPreviousError(&pid, 0.25f));

    float output = VOLOOP_PID_Compute(&pid, 3.0f, 1.0f);

    TEST_EXPECT_FLOAT_NEAR(ctx, 2.0f, pid.PreviousError);
    TEST_EXPECT_FLOAT_NEAR(ctx, 3.0f, pid.Integral);
    TEST_EXPECT_FLOAT_NEAR(ctx, 5.9375f, output);
    TEST_EXPECT_STATE_EQ(ctx, PID_UnSaturated, VOLOOP_PID_GetState(&pid));
}

/* Verify that continuous PID initialization matches the current discrete conversion formula. */
static void test_continue_init_converts_to_discrete_gains(VoloopTestContext* ctx) {
    PID_HandleTypeDef pid = {0};
    PID_InitTypeDef init = {0};

    init.mode = PID_Continue;
    init.init.Continue.Kp = 1.5f;
    init.init.Continue.Ki = 20.0f;
    init.init.Continue.Kd = 0.25f;
    init.init.Continue.triggerFrequency = 1000U;

    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PID_Init(&pid, &init));
    TEST_EXPECT_FLOAT_NEAR(ctx, 1.5f, pid.KpDiscrete);
    TEST_EXPECT_FLOAT_NEAR(ctx, 0.02f, pid.KiDiscrete);
    TEST_EXPECT_FLOAT_NEAR(ctx, 250.0f, pid.KdDiscrete);
}

/* Verify that one-zero initialization matches the current discrete conversion formula. */
static void test_one_zero_init_converts_to_discrete_gains(VoloopTestContext* ctx) {
    PID_HandleTypeDef pid = {0};
    PID_InitTypeDef init = {0};

    init.mode = PID_OneZero;
    init.init.OneZero.gain = 2.0f;
    init.init.OneZero.zero = 50.0f;
    init.init.OneZero.triggerFrequency = 1000U;

    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PID_Init(&pid, &init));
    TEST_EXPECT_FLOAT_NEAR(ctx, 2.0f, pid.KpDiscrete);
    TEST_EXPECT_FLOAT_NEAR(ctx, VOLOOP_TwoPi * 2.0f * 50.0f / 1000.0f, pid.KiDiscrete);
    TEST_EXPECT_FLOAT_NEAR(ctx, 0.0f, pid.KdDiscrete);
}

/* Verify that two-zero initialization matches the current discrete conversion formula. */
static void test_two_zero_init_converts_to_discrete_gains(VoloopTestContext* ctx) {
    PID_HandleTypeDef pid = {0};
    PID_InitTypeDef init = {0};

    init.mode = PID_TwoZero;
    init.init.TwoZero.gain = 0.5f;
    init.init.TwoZero.zero1 = 20.0f;
    init.init.TwoZero.zero2 = 40.0f;
    init.init.TwoZero.triggerFrequency = 2000U;

    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PID_Init(&pid, &init));
    TEST_EXPECT_FLOAT_NEAR(ctx, VOLOOP_TwoPi * 0.5f * (20.0f + 40.0f), pid.KpDiscrete);
    TEST_EXPECT_FLOAT_NEAR(ctx, VOLOOP_FourPiSquared * 0.5f * 20.0f * 40.0f / 2000.0f,
                           pid.KiDiscrete);
    TEST_EXPECT_FLOAT_NEAR(ctx, 1000.0f, pid.KdDiscrete);
}

/* Verify conditional integration upper saturation clamps output and freezes integral. */
static void test_conditional_integration_upper_saturation_contract(VoloopTestContext* ctx) {
    PID_HandleTypeDef pid = {0};
    PID_InitTypeDef init = make_discrete_init(1.0f, 1.0f, 0.0f);

    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PID_Init(&pid, &init));
    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PID_SetIntegral(&pid, 2.0f));

    float output = VOLOOP_PID_ComputeConditional(&pid, 5.0f, 1.0f, -10.0f, 3.0f);

    TEST_EXPECT_FLOAT_NEAR(ctx, 3.0f, output);
    TEST_EXPECT_FLOAT_NEAR(ctx, 2.0f, pid.Integral);
    TEST_EXPECT_FLOAT_NEAR(ctx, 4.0f, pid.PreviousError);
    TEST_EXPECT_STATE_EQ(ctx, PID_UpperSaturated, VOLOOP_PID_GetState(&pid));
}

/* Verify conditional integration lower saturation clamps output and freezes integral. */
static void test_conditional_integration_lower_saturation_contract(VoloopTestContext* ctx) {
    PID_HandleTypeDef pid = {0};
    PID_InitTypeDef init = make_discrete_init(1.0f, 1.0f, 0.0f);

    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PID_Init(&pid, &init));
    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PID_SetIntegral(&pid, -2.0f));

    float output = VOLOOP_PID_ComputeConditional(&pid, 1.0f, 5.0f, -3.0f, 10.0f);

    TEST_EXPECT_FLOAT_NEAR(ctx, -3.0f, output);
    TEST_EXPECT_FLOAT_NEAR(ctx, -2.0f, pid.Integral);
    TEST_EXPECT_FLOAT_NEAR(ctx, -4.0f, pid.PreviousError);
    TEST_EXPECT_STATE_EQ(ctx, PID_LowerSaturated, VOLOOP_PID_GetState(&pid));
}

/* Verify back-calculation clamps upper output and applies the current integral correction. */
static void test_back_calculation_upper_saturation_contract(VoloopTestContext* ctx) {
    PID_HandleTypeDef pid = {0};
    PID_InitTypeDef init = make_discrete_init(1.0f, 1.0f, 0.0f);

    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PID_Init(&pid, &init));
    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PID_SetIntegral(&pid, 2.0f));

    float output = VOLOOP_PID_ComputeBackCalculation(&pid, 5.0f, 1.0f, -10.0f, 3.0f, 0.5f);

    TEST_EXPECT_FLOAT_NEAR(ctx, 3.0f, output);
    TEST_EXPECT_FLOAT_NEAR(ctx, 4.5f, pid.Integral);
    TEST_EXPECT_FLOAT_NEAR(ctx, 4.0f, pid.PreviousError);
    TEST_EXPECT_STATE_EQ(ctx, PID_UpperSaturated, VOLOOP_PID_GetState(&pid));
}

/* Verify back-calculation clamps lower output and applies the current integral correction. */
static void test_back_calculation_lower_saturation_contract(VoloopTestContext* ctx) {
    PID_HandleTypeDef pid = {0};
    PID_InitTypeDef init = make_discrete_init(1.0f, 1.0f, 0.0f);

    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PID_Init(&pid, &init));
    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PID_SetIntegral(&pid, -2.0f));

    float output = VOLOOP_PID_ComputeBackCalculation(&pid, 1.0f, 5.0f, -3.0f, 10.0f, 0.5f);

    TEST_EXPECT_FLOAT_NEAR(ctx, -3.0f, output);
    TEST_EXPECT_FLOAT_NEAR(ctx, -4.5f, pid.Integral);
    TEST_EXPECT_FLOAT_NEAR(ctx, -4.0f, pid.PreviousError);
    TEST_EXPECT_STATE_EQ(ctx, PID_LowerSaturated, VOLOOP_PID_GetState(&pid));
}

int main(void) {
    VoloopTestContext ctx = VOLOOP_TEST_CONTEXT_INIT;

    voloop_run_test(&ctx, "discrete init copies gains", test_discrete_init_copies_gains);
    voloop_run_test(&ctx, "compute updates state by difference equation",
                    test_compute_updates_state_by_difference_equation);
    voloop_run_test(&ctx, "continue init converts to discrete gains",
                    test_continue_init_converts_to_discrete_gains);
    voloop_run_test(&ctx, "one-zero init converts to discrete gains",
                    test_one_zero_init_converts_to_discrete_gains);
    voloop_run_test(&ctx, "two-zero init converts to discrete gains",
                    test_two_zero_init_converts_to_discrete_gains);
    voloop_run_test(&ctx, "conditional integration upper saturation",
                    test_conditional_integration_upper_saturation_contract);
    voloop_run_test(&ctx, "conditional integration lower saturation",
                    test_conditional_integration_lower_saturation_contract);
    voloop_run_test(&ctx, "back-calculation upper saturation",
                    test_back_calculation_upper_saturation_contract);
    voloop_run_test(&ctx, "back-calculation lower saturation",
                    test_back_calculation_lower_saturation_contract);

    return voloop_test_report(&ctx);
}
