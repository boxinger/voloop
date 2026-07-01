#include "voloop_pid.h"
#include "voloop_test.h"
#include "voloop_test_float.h"

#define TEST_TOLERANCE 1.0e-5f

#define TEST_ASSERT_STATUS_EQ(expected, actual) TEST_ASSERT_INT_EQ((expected), (actual))
#define TEST_ASSERT_STATE_EQ(expected, actual) TEST_ASSERT_INT_EQ((expected), (actual))

static PID_InitTypeDef make_discrete_init(float kp, float ki, float kd) {
    PID_InitTypeDef init = {0};

    init.mode = PID_Discrete;
    init.init.Discrete.KpDiscrete = kp;
    init.init.Discrete.KiDiscrete = ki;
    init.init.Discrete.KdDiscrete = kd;

    return init;
}

/* Verify that discrete PID initialization copies Kp, Ki, and Kd into the handle. */
static int test_discrete_init_copies_gains(void) {
    PID_HandleTypeDef pid = {0};
    PID_InitTypeDef init = make_discrete_init(1.25f, 0.5f, 0.125f);

    TEST_ASSERT_STATUS_EQ(VOLOOP_OK, VOLOOP_PID_Init(&pid, &init));
    TEST_ASSERT_FLOAT_NEAR(1.25f, pid.KpDiscrete, TEST_TOLERANCE);
    TEST_ASSERT_FLOAT_NEAR(0.5f, pid.KiDiscrete, TEST_TOLERANCE);
    TEST_ASSERT_FLOAT_NEAR(0.125f, pid.KdDiscrete, TEST_TOLERANCE);

    return 0;
}

/* Verify the plain compute contract for error, integral, derivative, output, and history. */
static int test_compute_updates_state_by_difference_equation(void) {
    PID_HandleTypeDef pid = {0};
    PID_InitTypeDef init = make_discrete_init(2.0f, 0.5f, 0.25f);

    TEST_ASSERT_STATUS_EQ(VOLOOP_OK, VOLOOP_PID_Init(&pid, &init));
    TEST_ASSERT_STATUS_EQ(VOLOOP_OK, VOLOOP_PID_SetIntegral(&pid, 1.0f));
    TEST_ASSERT_STATUS_EQ(VOLOOP_OK, VOLOOP_PID_SetPreviousError(&pid, 0.25f));

    float output = VOLOOP_PID_Compute(&pid, 3.0f, 1.0f);

    TEST_ASSERT_FLOAT_NEAR(2.0f, pid.PreviousError, TEST_TOLERANCE);
    TEST_ASSERT_FLOAT_NEAR(3.0f, pid.Integral, TEST_TOLERANCE);
    TEST_ASSERT_FLOAT_NEAR(5.9375f, output, TEST_TOLERANCE);
    TEST_ASSERT_STATE_EQ(PID_UnSaturated, VOLOOP_PID_GetState(&pid));

    return 0;
}

/* Verify that continuous PID initialization matches the current discrete conversion formula. */
static int test_continue_init_converts_to_discrete_gains(void) {
    PID_HandleTypeDef pid = {0};
    PID_InitTypeDef init = {0};

    init.mode = PID_Continue;
    init.init.Continue.Kp = 1.5f;
    init.init.Continue.Ki = 20.0f;
    init.init.Continue.Kd = 0.25f;
    init.init.Continue.triggerFrequency = 1000U;

    TEST_ASSERT_STATUS_EQ(VOLOOP_OK, VOLOOP_PID_Init(&pid, &init));
    TEST_ASSERT_FLOAT_NEAR(1.5f, pid.KpDiscrete, TEST_TOLERANCE);
    TEST_ASSERT_FLOAT_NEAR(0.02f, pid.KiDiscrete, TEST_TOLERANCE);
    TEST_ASSERT_FLOAT_NEAR(250.0f, pid.KdDiscrete, TEST_TOLERANCE);

    return 0;
}

/* Verify that one-zero initialization matches the current discrete conversion formula. */
static int test_one_zero_init_converts_to_discrete_gains(void) {
    PID_HandleTypeDef pid = {0};
    PID_InitTypeDef init = {0};

    init.mode = PID_OneZero;
    init.init.OneZero.gain = 2.0f;
    init.init.OneZero.zero = 50.0f;
    init.init.OneZero.triggerFrequency = 1000U;

    TEST_ASSERT_STATUS_EQ(VOLOOP_OK, VOLOOP_PID_Init(&pid, &init));
    TEST_ASSERT_FLOAT_NEAR(2.0f, pid.KpDiscrete, TEST_TOLERANCE);
    TEST_ASSERT_FLOAT_NEAR(VOLOOP_TwoPi * 2.0f * 50.0f / 1000.0f, pid.KiDiscrete,
                           TEST_TOLERANCE);
    TEST_ASSERT_FLOAT_NEAR(0.0f, pid.KdDiscrete, TEST_TOLERANCE);

    return 0;
}

/* Verify that two-zero initialization matches the current discrete conversion formula. */
static int test_two_zero_init_converts_to_discrete_gains(void) {
    PID_HandleTypeDef pid = {0};
    PID_InitTypeDef init = {0};

    init.mode = PID_TwoZero;
    init.init.TwoZero.gain = 0.5f;
    init.init.TwoZero.zero1 = 20.0f;
    init.init.TwoZero.zero2 = 40.0f;
    init.init.TwoZero.triggerFrequency = 2000U;

    TEST_ASSERT_STATUS_EQ(VOLOOP_OK, VOLOOP_PID_Init(&pid, &init));
    TEST_ASSERT_FLOAT_NEAR(VOLOOP_TwoPi * 0.5f * (20.0f + 40.0f), pid.KpDiscrete,
                           TEST_TOLERANCE);
    TEST_ASSERT_FLOAT_NEAR(VOLOOP_FourPiSquared * 0.5f * 20.0f * 40.0f / 2000.0f,
                           pid.KiDiscrete, TEST_TOLERANCE);
    TEST_ASSERT_FLOAT_NEAR(1000.0f, pid.KdDiscrete, TEST_TOLERANCE);

    return 0;
}

/* Verify conditional integration upper saturation clamps output and freezes integral. */
static int test_conditional_integration_upper_saturation_contract(void) {
    PID_HandleTypeDef pid = {0};
    PID_InitTypeDef init = make_discrete_init(1.0f, 1.0f, 0.0f);

    TEST_ASSERT_STATUS_EQ(VOLOOP_OK, VOLOOP_PID_Init(&pid, &init));
    TEST_ASSERT_STATUS_EQ(VOLOOP_OK, VOLOOP_PID_SetIntegral(&pid, 2.0f));

    float output = VOLOOP_PID_ComputeConditional(&pid, 5.0f, 1.0f, -10.0f, 3.0f);

    TEST_ASSERT_FLOAT_NEAR(3.0f, output, TEST_TOLERANCE);
    TEST_ASSERT_FLOAT_NEAR(2.0f, pid.Integral, TEST_TOLERANCE);
    TEST_ASSERT_FLOAT_NEAR(4.0f, pid.PreviousError, TEST_TOLERANCE);
    TEST_ASSERT_STATE_EQ(PID_UpperSaturated, VOLOOP_PID_GetState(&pid));

    return 0;
}

/* Verify conditional integration lower saturation clamps output and freezes integral. */
static int test_conditional_integration_lower_saturation_contract(void) {
    PID_HandleTypeDef pid = {0};
    PID_InitTypeDef init = make_discrete_init(1.0f, 1.0f, 0.0f);

    TEST_ASSERT_STATUS_EQ(VOLOOP_OK, VOLOOP_PID_Init(&pid, &init));
    TEST_ASSERT_STATUS_EQ(VOLOOP_OK, VOLOOP_PID_SetIntegral(&pid, -2.0f));

    float output = VOLOOP_PID_ComputeConditional(&pid, 1.0f, 5.0f, -3.0f, 10.0f);

    TEST_ASSERT_FLOAT_NEAR(-3.0f, output, TEST_TOLERANCE);
    TEST_ASSERT_FLOAT_NEAR(-2.0f, pid.Integral, TEST_TOLERANCE);
    TEST_ASSERT_FLOAT_NEAR(-4.0f, pid.PreviousError, TEST_TOLERANCE);
    TEST_ASSERT_STATE_EQ(PID_LowerSaturated, VOLOOP_PID_GetState(&pid));

    return 0;
}

/* Verify back-calculation clamps upper output and applies the current integral correction. */
static int test_back_calculation_upper_saturation_contract(void) {
    PID_HandleTypeDef pid = {0};
    PID_InitTypeDef init = make_discrete_init(1.0f, 1.0f, 0.0f);

    TEST_ASSERT_STATUS_EQ(VOLOOP_OK, VOLOOP_PID_Init(&pid, &init));
    TEST_ASSERT_STATUS_EQ(VOLOOP_OK, VOLOOP_PID_SetIntegral(&pid, 2.0f));

    float output = VOLOOP_PID_ComputeBackCalculation(&pid, 5.0f, 1.0f, -10.0f, 3.0f, 0.5f);

    TEST_ASSERT_FLOAT_NEAR(3.0f, output, TEST_TOLERANCE);
    TEST_ASSERT_FLOAT_NEAR(4.5f, pid.Integral, TEST_TOLERANCE);
    TEST_ASSERT_FLOAT_NEAR(4.0f, pid.PreviousError, TEST_TOLERANCE);
    TEST_ASSERT_STATE_EQ(PID_UpperSaturated, VOLOOP_PID_GetState(&pid));

    return 0;
}

/* Verify back-calculation clamps lower output and applies the current integral correction. */
static int test_back_calculation_lower_saturation_contract(void) {
    PID_HandleTypeDef pid = {0};
    PID_InitTypeDef init = make_discrete_init(1.0f, 1.0f, 0.0f);

    TEST_ASSERT_STATUS_EQ(VOLOOP_OK, VOLOOP_PID_Init(&pid, &init));
    TEST_ASSERT_STATUS_EQ(VOLOOP_OK, VOLOOP_PID_SetIntegral(&pid, -2.0f));

    float output = VOLOOP_PID_ComputeBackCalculation(&pid, 1.0f, 5.0f, -3.0f, 10.0f, 0.5f);

    TEST_ASSERT_FLOAT_NEAR(-3.0f, output, TEST_TOLERANCE);
    TEST_ASSERT_FLOAT_NEAR(-4.5f, pid.Integral, TEST_TOLERANCE);
    TEST_ASSERT_FLOAT_NEAR(-4.0f, pid.PreviousError, TEST_TOLERANCE);
    TEST_ASSERT_STATE_EQ(PID_LowerSaturated, VOLOOP_PID_GetState(&pid));

    return 0;
}

int main(void) {
    if (voloop_run_test("discrete init copies gains", test_discrete_init_copies_gains) != 0) {
        return 1;
    }
    if (voloop_run_test("compute updates state by difference equation",
                        test_compute_updates_state_by_difference_equation) != 0) {
        return 1;
    }
    if (voloop_run_test("continue init converts to discrete gains",
                        test_continue_init_converts_to_discrete_gains) != 0) {
        return 1;
    }
    if (voloop_run_test("one-zero init converts to discrete gains",
                        test_one_zero_init_converts_to_discrete_gains) != 0) {
        return 1;
    }
    if (voloop_run_test("two-zero init converts to discrete gains",
                        test_two_zero_init_converts_to_discrete_gains) != 0) {
        return 1;
    }
    if (voloop_run_test("conditional integration upper saturation",
                        test_conditional_integration_upper_saturation_contract) != 0) {
        return 1;
    }
    if (voloop_run_test("conditional integration lower saturation",
                        test_conditional_integration_lower_saturation_contract) != 0) {
        return 1;
    }
    if (voloop_run_test("back-calculation upper saturation",
                        test_back_calculation_upper_saturation_contract) != 0) {
        return 1;
    }
    if (voloop_run_test("back-calculation lower saturation",
                        test_back_calculation_lower_saturation_contract) != 0) {
        return 1;
    }

    return 0;
}
