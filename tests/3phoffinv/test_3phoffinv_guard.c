#include "voloop_3phOffInv.h"
#include "voloop_test.h"
#include "voloop_test_float.h"

#include <math.h>

#define TEST_EXPECT_STATUS_EQ(ctx, expected, actual)                                               \
    VOLOOP_EXPECT_EQ_INT((ctx), (expected), (actual))
#define TEST_REQUIRE_STATUS_EQ(ctx, expected, actual)                                              \
    VOLOOP_REQUIRE_EQ_INT((ctx), (expected), (actual))
#define TEST_EXPECT_STATE_EQ(ctx, expected, actual)                                                \
    VOLOOP_EXPECT_EQ_INT((ctx), (expected), (actual))

static NCO_InitTypeDef make_nco_init(float initialRad) {
    NCO_InitTypeDef init = {
        .triggerFrequency = 1000U,
        .initialFrequency = 50.0f,
        .initialRad = initialRad,
    };
    return init;
}

static ThreePhOffInv_ConfigTypeDef make_config(void) {
    ThreePhOffInv_ConfigTypeDef config = {
        .LineOverVoltageThreshold = 500.0f,
        .PhaseOverCurrentThreshold = 20.0f,
        .BusUnderVoltageThreshold = 300.0f,
        .BusOverVoltageThreshold = 450.0f,
    };
    return config;
}

static ThreePhOffInv_InputTypeDef make_valid_input(void) {
    ThreePhOffInv_InputTypeDef input = {
        .BusVoltage = 400.0f,
        .LineVoltageAB = 0.0f,
        .LineVoltageAC = 0.0f,
        .PhaseCurrentA = 0.0f,
        .PhaseCurrentB = 0.0f,
    };
    return input;
}

static void init_inverter(VoloopTestContext* ctx, ThreePhOffInv_HandleTypeDef* handle) {
    NCO_InitTypeDef ncoInit = make_nco_init(0.0f);
    ThreePhOffInv_ConfigTypeDef config = make_config();
    ThreePhOffInv_InitTypeDef init = {
        .NCOInit = &ncoInit,
        .Config = &config,
    };

    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_3phOffInv_Init(handle, &init));
}

static void expect_output_disabled(VoloopTestContext* ctx,
                                   const ThreePhOffInv_OutputTypeDef* output) {
    TEST_EXPECT_STATE_EQ(ctx, VOLOOP_PWM_DISABLED, output->PhaseAPwmState);
    TEST_EXPECT_STATE_EQ(ctx, VOLOOP_PWM_DISABLED, output->PhaseBPwmState);
    TEST_EXPECT_STATE_EQ(ctx, VOLOOP_PWM_DISABLED, output->PhaseCPwmState);
    VOLOOP_EXPECT_FLOAT_NEAR(ctx, 0.0f, output->PhaseADuty, 0.0f);
    VOLOOP_EXPECT_FLOAT_NEAR(ctx, 0.0f, output->PhaseBDuty, 0.0f);
    VOLOOP_EXPECT_FLOAT_NEAR(ctx, 0.0f, output->PhaseCDuty, 0.0f);
}

static void expect_fault(VoloopTestContext* ctx, ThreePhOffInv_InputTypeDef input,
                         ThreePhOffInv_FaultCodeTypeDef expectedFault) {
    ThreePhOffInv_HandleTypeDef handle = { 0 };
    ThreePhOffInv_OutputTypeDef output = { 0 };

    init_inverter(ctx, &handle);
    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_3phOffInv_Start(&handle));
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_ERROR, VOLOOP_3phOffInv_Sync(&handle, &input, &output));
    TEST_EXPECT_STATE_EQ(ctx, THREEPHOFFINV_ERROR, VOLOOP_3phOffInv_GetState(&handle));
    TEST_EXPECT_STATE_EQ(ctx, expectedFault, VOLOOP_3phOffInv_GetFaultCode(&handle));
    TEST_EXPECT_STATE_EQ(ctx, NCO_STOPPED, VOLOOP_NCO_GetState(&handle.NCO));
    expect_output_disabled(ctx, &output);
}

static void test_sync_guards_arguments_and_disabled_state(VoloopTestContext* ctx) {
    ThreePhOffInv_HandleTypeDef handle = { 0 };
    ThreePhOffInv_InputTypeDef input = make_valid_input();
    ThreePhOffInv_OutputTypeDef output = {
        .PhaseAPwmState = VOLOOP_PWM_ENABLE,
        .PhaseADuty = 1.0f,
        .PhaseBPwmState = VOLOOP_PWM_ENABLE,
        .PhaseBDuty = 1.0f,
        .PhaseCPwmState = VOLOOP_PWM_ENABLE,
        .PhaseCDuty = 1.0f,
    };

    init_inverter(ctx, &handle);
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_3phOffInv_Sync(NULL, &input, &output));
    expect_output_disabled(ctx, &output);
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_3phOffInv_Sync(&handle, NULL, &output));
    expect_output_disabled(ctx, &output);
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_3phOffInv_Sync(&handle, &input, NULL));
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_3phOffInv_Sync(&handle, &input, &output));
    expect_output_disabled(ctx, &output);
}

static void test_sync_latches_sample_and_reconstructed_protection_faults(VoloopTestContext* ctx) {
    ThreePhOffInv_InputTypeDef input = make_valid_input();

    input.LineVoltageAB = NAN;
    expect_fault(ctx, input, THREEPHOFFINV_FAULT_INVALID_SAMPLE);

    input = make_valid_input();
    input.LineVoltageAB = -300.0f;
    input.LineVoltageAC = 300.0f;
    expect_fault(ctx, input, THREEPHOFFINV_FAULT_LINE_OVERVOLTAGE);

    input = make_valid_input();
    input.PhaseCurrentA = 15.0f;
    input.PhaseCurrentB = 15.0f;
    expect_fault(ctx, input, THREEPHOFFINV_FAULT_PHASE_OVERCURRENT);

    input = make_valid_input();
    input.BusVoltage = 299.0f;
    expect_fault(ctx, input, THREEPHOFFINV_FAULT_BUS_UNDERVOLTAGE);

    input = make_valid_input();
    input.BusVoltage = 451.0f;
    expect_fault(ctx, input, THREEPHOFFINV_FAULT_BUS_OVERVOLTAGE);
}

static void test_fault_can_be_cleared_without_restarting_output(VoloopTestContext* ctx) {
    ThreePhOffInv_HandleTypeDef handle = { 0 };
    ThreePhOffInv_InputTypeDef input = make_valid_input();
    ThreePhOffInv_OutputTypeDef output = { 0 };

    init_inverter(ctx, &handle);
    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_3phOffInv_Start(&handle));
    input.BusVoltage = 299.0f;
    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_ERROR, VOLOOP_3phOffInv_Sync(&handle, &input, &output));
    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_3phOffInv_ClearFaultCode(&handle));
    TEST_EXPECT_STATE_EQ(ctx, THREEPHOFFINV_DISABLED, VOLOOP_3phOffInv_GetState(&handle));
    TEST_EXPECT_STATE_EQ(ctx, THREEPHOFFINV_FAULT_NONE, VOLOOP_3phOffInv_GetFaultCode(&handle));
    TEST_EXPECT_STATE_EQ(ctx, NCO_STOPPED, VOLOOP_NCO_GetState(&handle.NCO));
}

int main(void) {
    VoloopTestContext ctx = VOLOOP_TEST_CONTEXT_INIT;

    voloop_run_test(&ctx, "sync guards arguments and disabled state",
                    test_sync_guards_arguments_and_disabled_state);
    voloop_run_test(&ctx, "sync latches sample and protection faults",
                    test_sync_latches_sample_and_reconstructed_protection_faults);
    voloop_run_test(&ctx, "fault clears without restart",
                    test_fault_can_be_cleared_without_restarting_output);

    return voloop_test_report(&ctx);
}
