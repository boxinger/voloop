#include "voloop_pid.h"
#include "voloop_test.h"
#include "voloop_test_float.h"

#define TEST_EXPECT_STATUS_EQ(ctx, expected, actual) VOLOOP_EXPECT_EQ_INT((ctx), (expected), (actual))
#define TEST_REQUIRE_STATUS_EQ(ctx, expected, actual) VOLOOP_REQUIRE_EQ_INT((ctx), (expected), (actual))
#define TEST_EXPECT_STATE_EQ(ctx, expected, actual) VOLOOP_EXPECT_EQ_INT((ctx), (expected), (actual))
#define TEST_EXPECT_FLOAT_EQ(ctx, expected, actual) VOLOOP_EXPECT_FLOAT_NEAR((ctx), (expected), (actual), 0.0f)

static PID_InitTypeDef make_discrete_init(void) {
    PID_InitTypeDef init = {0};

    init.mode = PID_Discrete;
    init.init.Discrete.KpDiscrete = 1.0f;
    init.init.Discrete.KiDiscrete = 0.25f;
    init.init.Discrete.KdDiscrete = 0.125f;

    return init;
}

/* Validate that PID init rejects NULL handle and NULL init arguments. */
static void test_init_rejects_null_arguments(VoloopTestContext* ctx) {
    PID_HandleTypeDef pid = {0};
    PID_InitTypeDef init = make_discrete_init();

    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PID_Init(NULL, &init));
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PID_Init(&pid, NULL));
}

/* Validate that PID init rejects unknown initialization modes. */
static void test_init_rejects_invalid_mode(VoloopTestContext* ctx) {
    PID_HandleTypeDef pid = {0};
    PID_InitTypeDef init = make_discrete_init();

    init.mode = (PID_InitModeTypeDef)255;

    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PID_Init(&pid, &init));
}

/* Validate that frequency-based PID init modes reject a zero trigger frequency. */
static void test_init_rejects_zero_trigger_frequency(VoloopTestContext* ctx) {
    PID_HandleTypeDef pid = {0};
    PID_InitTypeDef init = {0};

    init.mode = PID_Continue;
    init.init.Continue.triggerFrequency = 0U;
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PID_Init(&pid, &init));

    init = (PID_InitTypeDef){0};
    init.mode = PID_OneZero;
    init.init.OneZero.triggerFrequency = 0U;
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PID_Init(&pid, &init));

    init = (PID_InitTypeDef){0};
    init.mode = PID_TwoZero;
    init.init.TwoZero.triggerFrequency = 0U;
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PID_Init(&pid, &init));
}

/* Validate that valid discrete init succeeds and clears runtime history. */
static void test_discrete_init_sets_unsaturated_state_and_clears_history(VoloopTestContext* ctx) {
    PID_HandleTypeDef pid = {
        .Integral = 12.0f,
        .PreviousError = -3.0f,
        .State = PID_ERROR,
    };
    PID_InitTypeDef init = make_discrete_init();

    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PID_Init(&pid, &init));
    TEST_EXPECT_STATE_EQ(ctx, PID_UnSaturated, VOLOOP_PID_GetState(&pid));
    TEST_EXPECT_FLOAT_EQ(ctx, 0.0f, pid.Integral);
    TEST_EXPECT_FLOAT_EQ(ctx, 0.0f, pid.PreviousError);
}

/* Validate that PID deinit rejects NULL and moves a valid handle to PID_ERROR. */
static void test_deinit_rejects_null_and_sets_error_state(VoloopTestContext* ctx) {
    PID_HandleTypeDef pid = {0};
    PID_InitTypeDef init = make_discrete_init();

    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PID_DeInit(NULL));
    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PID_Init(&pid, &init));
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PID_DeInit(&pid));
    TEST_EXPECT_STATE_EQ(ctx, PID_ERROR, VOLOOP_PID_GetState(&pid));
}

/* Validate that PID reset rejects NULL and clears history on a valid handle. */
static void test_reset_rejects_null_and_clears_history(VoloopTestContext* ctx) {
    PID_HandleTypeDef pid = {
        .Integral = 4.0f,
        .PreviousError = -2.0f,
        .State = PID_UpperSaturated,
    };

    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PID_Reset(NULL));
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PID_Reset(&pid));
    TEST_EXPECT_FLOAT_EQ(ctx, 0.0f, pid.Integral);
    TEST_EXPECT_FLOAT_EQ(ctx, 0.0f, pid.PreviousError);
    TEST_EXPECT_STATE_EQ(ctx, PID_UnSaturated, VOLOOP_PID_GetState(&pid));
}

/* Validate that PID setters reject NULL handles and update valid handles. */
static void test_setters_reject_null_and_write_values(VoloopTestContext* ctx) {
    PID_HandleTypeDef pid = {0};

    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PID_SetIntegral(NULL, 1.5f));
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_PID_SetPreviousError(NULL, -2.5f));

    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PID_SetIntegral(&pid, 1.5f));
    TEST_EXPECT_FLOAT_EQ(ctx, 1.5f, pid.Integral);

    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_PID_SetPreviousError(&pid, -2.5f));
    TEST_EXPECT_FLOAT_EQ(ctx, -2.5f, pid.PreviousError);
}

/* Validate that PID state and compute APIs return safe defaults for NULL handles. */
static void test_get_state_and_compute_reject_null_safely(VoloopTestContext* ctx) {
    TEST_EXPECT_STATE_EQ(ctx, PID_ERROR, VOLOOP_PID_GetState(NULL));
    TEST_EXPECT_FLOAT_EQ(ctx, 0.0f, VOLOOP_PID_Compute(NULL, 1.0f, 0.5f));
    TEST_EXPECT_FLOAT_EQ(ctx, 0.0f, VOLOOP_PID_ComputeConditional(NULL, 1.0f, 0.5f, 0.0f, 1.0f));
    TEST_EXPECT_FLOAT_EQ(ctx, 0.0f,
                         VOLOOP_PID_ComputeBackCalculation(NULL, 1.0f, 0.5f, 0.0f, 1.0f, 0.1f));
}

int main(void) {
    VoloopTestContext ctx = VOLOOP_TEST_CONTEXT_INIT;

    voloop_run_test(&ctx, "init rejects null arguments", test_init_rejects_null_arguments);
    voloop_run_test(&ctx, "init rejects invalid mode", test_init_rejects_invalid_mode);
    voloop_run_test(&ctx, "init rejects zero trigger frequency", test_init_rejects_zero_trigger_frequency);
    voloop_run_test(&ctx, "discrete init clears history",
                    test_discrete_init_sets_unsaturated_state_and_clears_history);
    voloop_run_test(&ctx, "deinit rejects null and sets error state",
                    test_deinit_rejects_null_and_sets_error_state);
    voloop_run_test(&ctx, "reset rejects null and clears history", test_reset_rejects_null_and_clears_history);
    voloop_run_test(&ctx, "setters reject null and write values", test_setters_reject_null_and_write_values);
    voloop_run_test(&ctx, "get state and compute reject null safely",
                    test_get_state_and_compute_reject_null_safely);

    return voloop_test_report(&ctx);
}
