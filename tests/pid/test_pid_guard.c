#include "voloop_pid.h"
#include "voloop_test.h"
#include "voloop_test_float.h"

#define TEST_ASSERT_STATUS_EQ(expected, actual) TEST_ASSERT_INT_EQ((expected), (actual))
#define TEST_ASSERT_STATE_EQ(expected, actual) TEST_ASSERT_INT_EQ((expected), (actual))
#define TEST_ASSERT_FLOAT_EQ(expected, actual) TEST_ASSERT_FLOAT_NEAR((expected), (actual), 0.0f)

static PID_InitTypeDef make_discrete_init(void) {
    PID_InitTypeDef init = {0};

    init.mode = PID_Discrete;
    init.init.Discrete.KpDiscrete = 1.0f;
    init.init.Discrete.KiDiscrete = 0.25f;
    init.init.Discrete.KdDiscrete = 0.125f;

    return init;
}

/* Validate that PID init rejects NULL handle and NULL init arguments. */
static int test_init_rejects_null_arguments(void) {
    PID_HandleTypeDef pid = {0};
    PID_InitTypeDef init = make_discrete_init();

    TEST_ASSERT_STATUS_EQ(VOLOOP_INVALID_PARAM, VOLOOP_PID_Init(NULL, &init));
    TEST_ASSERT_STATUS_EQ(VOLOOP_INVALID_PARAM, VOLOOP_PID_Init(&pid, NULL));

    return 0;
}

/* Validate that PID init rejects unknown initialization modes. */
static int test_init_rejects_invalid_mode(void) {
    PID_HandleTypeDef pid = {0};
    PID_InitTypeDef init = make_discrete_init();

    init.mode = (PID_InitModeTypeDef)255;

    TEST_ASSERT_STATUS_EQ(VOLOOP_INVALID_PARAM, VOLOOP_PID_Init(&pid, &init));

    return 0;
}

/* Validate that frequency-based PID init modes reject a zero trigger frequency. */
static int test_init_rejects_zero_trigger_frequency(void) {
    PID_HandleTypeDef pid = {0};
    PID_InitTypeDef init = {0};

    init.mode = PID_Continue;
    init.init.Continue.triggerFrequency = 0U;
    TEST_ASSERT_STATUS_EQ(VOLOOP_INVALID_PARAM, VOLOOP_PID_Init(&pid, &init));

    init = (PID_InitTypeDef){0};
    init.mode = PID_OneZero;
    init.init.OneZero.triggerFrequency = 0U;
    TEST_ASSERT_STATUS_EQ(VOLOOP_INVALID_PARAM, VOLOOP_PID_Init(&pid, &init));

    init = (PID_InitTypeDef){0};
    init.mode = PID_TwoZero;
    init.init.TwoZero.triggerFrequency = 0U;
    TEST_ASSERT_STATUS_EQ(VOLOOP_INVALID_PARAM, VOLOOP_PID_Init(&pid, &init));

    return 0;
}

/* Validate that valid discrete init succeeds and clears runtime history. */
static int test_discrete_init_sets_unsaturated_state_and_clears_history(void) {
    PID_HandleTypeDef pid = {
        .Integral = 12.0f,
        .PreviousError = -3.0f,
        .State = PID_ERROR,
    };
    PID_InitTypeDef init = make_discrete_init();

    TEST_ASSERT_STATUS_EQ(VOLOOP_OK, VOLOOP_PID_Init(&pid, &init));
    TEST_ASSERT_STATE_EQ(PID_UnSaturated, VOLOOP_PID_GetState(&pid));
    TEST_ASSERT_FLOAT_EQ(0.0f, pid.Integral);
    TEST_ASSERT_FLOAT_EQ(0.0f, pid.PreviousError);

    return 0;
}

/* Validate that PID deinit rejects NULL and moves a valid handle to PID_ERROR. */
static int test_deinit_rejects_null_and_sets_error_state(void) {
    PID_HandleTypeDef pid = {0};
    PID_InitTypeDef init = make_discrete_init();

    TEST_ASSERT_STATUS_EQ(VOLOOP_INVALID_PARAM, VOLOOP_PID_DeInit(NULL));
    TEST_ASSERT_STATUS_EQ(VOLOOP_OK, VOLOOP_PID_Init(&pid, &init));
    TEST_ASSERT_STATUS_EQ(VOLOOP_OK, VOLOOP_PID_DeInit(&pid));
    TEST_ASSERT_STATE_EQ(PID_ERROR, VOLOOP_PID_GetState(&pid));

    return 0;
}

/* Validate that PID reset rejects NULL and clears history on a valid handle. */
static int test_reset_rejects_null_and_clears_history(void) {
    PID_HandleTypeDef pid = {
        .Integral = 4.0f,
        .PreviousError = -2.0f,
        .State = PID_UpperSaturated,
    };

    TEST_ASSERT_STATUS_EQ(VOLOOP_INVALID_PARAM, VOLOOP_PID_Reset(NULL));
    TEST_ASSERT_STATUS_EQ(VOLOOP_OK, VOLOOP_PID_Reset(&pid));
    TEST_ASSERT_FLOAT_EQ(0.0f, pid.Integral);
    TEST_ASSERT_FLOAT_EQ(0.0f, pid.PreviousError);
    TEST_ASSERT_STATE_EQ(PID_UnSaturated, VOLOOP_PID_GetState(&pid));

    return 0;
}

/* Validate that PID setters reject NULL handles and update valid handles. */
static int test_setters_reject_null_and_write_values(void) {
    PID_HandleTypeDef pid = {0};

    TEST_ASSERT_STATUS_EQ(VOLOOP_INVALID_PARAM, VOLOOP_PID_SetIntegral(NULL, 1.5f));
    TEST_ASSERT_STATUS_EQ(VOLOOP_INVALID_PARAM, VOLOOP_PID_SetPreviousError(NULL, -2.5f));

    TEST_ASSERT_STATUS_EQ(VOLOOP_OK, VOLOOP_PID_SetIntegral(&pid, 1.5f));
    TEST_ASSERT_FLOAT_EQ(1.5f, pid.Integral);

    TEST_ASSERT_STATUS_EQ(VOLOOP_OK, VOLOOP_PID_SetPreviousError(&pid, -2.5f));
    TEST_ASSERT_FLOAT_EQ(-2.5f, pid.PreviousError);

    return 0;
}

/* Validate that PID state and compute APIs return safe defaults for NULL handles. */
static int test_get_state_and_compute_reject_null_safely(void) {
    TEST_ASSERT_STATE_EQ(PID_ERROR, VOLOOP_PID_GetState(NULL));
    TEST_ASSERT_FLOAT_EQ(0.0f, VOLOOP_PID_Compute(NULL, 1.0f, 0.5f));
    TEST_ASSERT_FLOAT_EQ(0.0f, VOLOOP_PID_ComputeConditional(NULL, 1.0f, 0.5f, 0.0f, 1.0f));
    TEST_ASSERT_FLOAT_EQ(0.0f,
                         VOLOOP_PID_ComputeBackCalculation(NULL, 1.0f, 0.5f, 0.0f, 1.0f, 0.1f));

    return 0;
}

int main(void) {
    if (voloop_run_test("init rejects null arguments", test_init_rejects_null_arguments) != 0) {
        return 1;
    }
    if (voloop_run_test("init rejects invalid mode", test_init_rejects_invalid_mode) != 0) {
        return 1;
    }
    if (voloop_run_test("init rejects zero trigger frequency",
                        test_init_rejects_zero_trigger_frequency) != 0) {
        return 1;
    }
    if (voloop_run_test("discrete init clears history",
                        test_discrete_init_sets_unsaturated_state_and_clears_history) != 0) {
        return 1;
    }
    if (voloop_run_test("deinit rejects null and sets error state",
                        test_deinit_rejects_null_and_sets_error_state) != 0) {
        return 1;
    }
    if (voloop_run_test("reset rejects null and clears history",
                        test_reset_rejects_null_and_clears_history) != 0) {
        return 1;
    }
    if (voloop_run_test("setters reject null and write values",
                        test_setters_reject_null_and_write_values) != 0) {
        return 1;
    }
    if (voloop_run_test("get state and compute reject null safely",
                        test_get_state_and_compute_reject_null_safely) != 0) {
        return 1;
    }

    return 0;
}
