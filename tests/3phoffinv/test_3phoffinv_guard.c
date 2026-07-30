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

static PID_InitTypeDef make_pid_init(void) {
    PID_InitTypeDef init = {
        .mode = PID_OneZero,
        .init.OneZero = {
            .gain = 0.001f,
            .zero = 20.0f,
            .triggerFrequency = 1000U,
        },
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
        .LineVoltageBC = 0.0f,
        .PhaseCurrentA = 0.0f,
        .PhaseCurrentC = 0.0f,
    };
    return input;
}

static void init_inverter(VoloopTestContext* ctx, ThreePhOffInv_HandleTypeDef* handle) {
    NCO_InitTypeDef ncoInit = make_nco_init(0.0f);
    PID_InitTypeDef voltageDInit = make_pid_init();
    PID_InitTypeDef voltageQInit = make_pid_init();
    PID_InitTypeDef currentDInit = make_pid_init();
    PID_InitTypeDef currentQInit = make_pid_init();
    ThreePhOffInv_ConfigTypeDef config = make_config();
    ThreePhOffInv_InitTypeDef init = {
        .NCOInit = &ncoInit,
        .Config = &config,
        .VoltageDControllerInit = &voltageDInit,
        .VoltageQControllerInit = &voltageQInit,
        .CurrentDControllerInit = &currentDInit,
        .CurrentQControllerInit = &currentQInit,
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
    input.LineVoltageAB = 300.0f;
    input.LineVoltageBC = 300.0f;
    expect_fault(ctx, input, THREEPHOFFINV_FAULT_LINE_OVERVOLTAGE);

    input = make_valid_input();
    input.PhaseCurrentA = NAN;
    expect_fault(ctx, input, THREEPHOFFINV_FAULT_INVALID_SAMPLE);

    input = make_valid_input();
    input.PhaseCurrentC = NAN;
    expect_fault(ctx, input, THREEPHOFFINV_FAULT_INVALID_SAMPLE);

    input = make_valid_input();
    input.PhaseCurrentA = 15.0f;
    input.PhaseCurrentC = 15.0f;
    expect_fault(ctx, input, THREEPHOFFINV_FAULT_PHASE_OVERCURRENT);

    input = make_valid_input();
    input.BusVoltage = 299.0f;
    expect_fault(ctx, input, THREEPHOFFINV_FAULT_BUS_UNDERVOLTAGE);

    input = make_valid_input();
    input.BusVoltage = 451.0f;
    expect_fault(ctx, input, THREEPHOFFINV_FAULT_BUS_OVERVOLTAGE);
}

static void test_zero_bus_voltage_latches_undervoltage_with_zero_threshold(
    VoloopTestContext* ctx) {
    ThreePhOffInv_HandleTypeDef handle = { 0 };
    NCO_InitTypeDef ncoInit = make_nco_init(0.0f);
    PID_InitTypeDef voltageDInit = make_pid_init();
    PID_InitTypeDef voltageQInit = make_pid_init();
    PID_InitTypeDef currentDInit = make_pid_init();
    PID_InitTypeDef currentQInit = make_pid_init();
    ThreePhOffInv_ConfigTypeDef config = make_config();
    config.BusUnderVoltageThreshold = 0.0f;
    ThreePhOffInv_InitTypeDef init = {
        .NCOInit = &ncoInit,
        .Config = &config,
        .VoltageDControllerInit = &voltageDInit,
        .VoltageQControllerInit = &voltageQInit,
        .CurrentDControllerInit = &currentDInit,
        .CurrentQControllerInit = &currentQInit,
    };
    ThreePhOffInv_InputTypeDef input = make_valid_input();
    input.BusVoltage = 0.0f;
    ThreePhOffInv_OutputTypeDef output = { 0 };

    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_3phOffInv_Init(&handle, &init));
    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_3phOffInv_Start(&handle));
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_ERROR,
                          VOLOOP_3phOffInv_Sync(&handle, &input, &output));
    TEST_EXPECT_STATE_EQ(ctx, THREEPHOFFINV_ERROR, handle.State);
    TEST_EXPECT_STATE_EQ(ctx, THREEPHOFFINV_FAULT_BUS_UNDERVOLTAGE, handle.FaultCode);
    expect_output_disabled(ctx, &output);
}

static void test_fault_can_be_cleared_without_restarting_output(VoloopTestContext* ctx) {
    ThreePhOffInv_HandleTypeDef handle = { 0 };
    ThreePhOffInv_InputTypeDef input = make_valid_input();
    ThreePhOffInv_OutputTypeDef output = { 0 };

    init_inverter(ctx, &handle);
    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_3phOffInv_Start(&handle));
    handle.VoltageDController.Integral = 12.0f;
    handle.VoltageDController.PreviousError = -3.0f;
    handle.VoltageDController.State = PID_UpperSaturated;
    handle.VoltageQController.Integral = -8.0f;
    handle.VoltageQController.PreviousError = 2.0f;
    handle.VoltageQController.State = PID_LowerSaturated;
    handle.CurrentDController.Integral = 6.0f;
    handle.CurrentDController.PreviousError = -1.0f;
    handle.CurrentDController.State = PID_UpperSaturated;
    handle.CurrentQController.Integral = -4.0f;
    handle.CurrentQController.PreviousError = 3.0f;
    handle.CurrentQController.State = PID_LowerSaturated;
    input.BusVoltage = 299.0f;
    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_ERROR, VOLOOP_3phOffInv_Sync(&handle, &input, &output));
    VOLOOP_EXPECT_FLOAT_NEAR(ctx, 12.0f, handle.VoltageDController.Integral, 0.0f);
    VOLOOP_EXPECT_FLOAT_NEAR(ctx, -3.0f, handle.VoltageDController.PreviousError, 0.0f);
    TEST_EXPECT_STATE_EQ(ctx, PID_UpperSaturated, handle.VoltageDController.State);
    VOLOOP_EXPECT_FLOAT_NEAR(ctx, -8.0f, handle.VoltageQController.Integral, 0.0f);
    VOLOOP_EXPECT_FLOAT_NEAR(ctx, 2.0f, handle.VoltageQController.PreviousError, 0.0f);
    TEST_EXPECT_STATE_EQ(ctx, PID_LowerSaturated, handle.VoltageQController.State);
    VOLOOP_EXPECT_FLOAT_NEAR(ctx, 6.0f, handle.CurrentDController.Integral, 0.0f);
    VOLOOP_EXPECT_FLOAT_NEAR(ctx, -1.0f, handle.CurrentDController.PreviousError, 0.0f);
    TEST_EXPECT_STATE_EQ(ctx, PID_UpperSaturated, handle.CurrentDController.State);
    VOLOOP_EXPECT_FLOAT_NEAR(ctx, -4.0f, handle.CurrentQController.Integral, 0.0f);
    VOLOOP_EXPECT_FLOAT_NEAR(ctx, 3.0f, handle.CurrentQController.PreviousError, 0.0f);
    TEST_EXPECT_STATE_EQ(ctx, PID_LowerSaturated, handle.CurrentQController.State);
    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_3phOffInv_ClearFaultCode(&handle));
    TEST_EXPECT_STATE_EQ(ctx, THREEPHOFFINV_DISABLED, VOLOOP_3phOffInv_GetState(&handle));
    TEST_EXPECT_STATE_EQ(ctx, THREEPHOFFINV_FAULT_NONE, VOLOOP_3phOffInv_GetFaultCode(&handle));
    TEST_EXPECT_STATE_EQ(ctx, NCO_STOPPED, VOLOOP_NCO_GetState(&handle.NCO));
    VOLOOP_EXPECT_FLOAT_NEAR(ctx, 0.0f, handle.VoltageDController.Integral, 0.0f);
    VOLOOP_EXPECT_FLOAT_NEAR(ctx, 0.0f, handle.VoltageDController.PreviousError, 0.0f);
    TEST_EXPECT_STATE_EQ(ctx, PID_UnSaturated, handle.VoltageDController.State);
    VOLOOP_EXPECT_FLOAT_NEAR(ctx, 0.0f, handle.VoltageQController.Integral, 0.0f);
    VOLOOP_EXPECT_FLOAT_NEAR(ctx, 0.0f, handle.VoltageQController.PreviousError, 0.0f);
    TEST_EXPECT_STATE_EQ(ctx, PID_UnSaturated, handle.VoltageQController.State);
    VOLOOP_EXPECT_FLOAT_NEAR(ctx, 0.0f, handle.CurrentDController.Integral, 0.0f);
    VOLOOP_EXPECT_FLOAT_NEAR(ctx, 0.0f, handle.CurrentDController.PreviousError, 0.0f);
    TEST_EXPECT_STATE_EQ(ctx, PID_UnSaturated, handle.CurrentDController.State);
    VOLOOP_EXPECT_FLOAT_NEAR(ctx, 0.0f, handle.CurrentQController.Integral, 0.0f);
    VOLOOP_EXPECT_FLOAT_NEAR(ctx, 0.0f, handle.CurrentQController.PreviousError, 0.0f);
    TEST_EXPECT_STATE_EQ(ctx, PID_UnSaturated, handle.CurrentQController.State);
}

static void test_controller_lifecycle(VoloopTestContext* ctx) {
    ThreePhOffInv_HandleTypeDef handle = { 0 };

    init_inverter(ctx, &handle);
    handle.VoltageDController.Integral = 4.0f;
    handle.VoltageQController.PreviousError = -5.0f;
    handle.CurrentDController.Integral = 3.0f;
    handle.CurrentQController.PreviousError = -2.0f;
    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_3phOffInv_Start(&handle));
    VOLOOP_EXPECT_FLOAT_NEAR(ctx, 0.0f, handle.VoltageDController.Integral, 0.0f);
    VOLOOP_EXPECT_FLOAT_NEAR(ctx, 0.0f, handle.VoltageQController.PreviousError, 0.0f);
    VOLOOP_EXPECT_FLOAT_NEAR(ctx, 0.0f, handle.CurrentDController.Integral, 0.0f);
    VOLOOP_EXPECT_FLOAT_NEAR(ctx, 0.0f, handle.CurrentQController.PreviousError, 0.0f);

    handle.VoltageDController.Integral = 6.0f;
    handle.VoltageQController.PreviousError = -7.0f;
    handle.CurrentDController.Integral = 5.0f;
    handle.CurrentQController.PreviousError = -4.0f;
    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_3phOffInv_Start(&handle));
    VOLOOP_EXPECT_FLOAT_NEAR(ctx, 6.0f, handle.VoltageDController.Integral, 0.0f);
    VOLOOP_EXPECT_FLOAT_NEAR(ctx, -7.0f, handle.VoltageQController.PreviousError, 0.0f);
    VOLOOP_EXPECT_FLOAT_NEAR(ctx, 5.0f, handle.CurrentDController.Integral, 0.0f);
    VOLOOP_EXPECT_FLOAT_NEAR(ctx, -4.0f, handle.CurrentQController.PreviousError, 0.0f);

    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_3phOffInv_Stop(&handle));
    VOLOOP_EXPECT_FLOAT_NEAR(ctx, 0.0f, handle.VoltageDController.Integral, 0.0f);
    VOLOOP_EXPECT_FLOAT_NEAR(ctx, 0.0f, handle.VoltageQController.PreviousError, 0.0f);
    VOLOOP_EXPECT_FLOAT_NEAR(ctx, 0.0f, handle.CurrentDController.Integral, 0.0f);
    VOLOOP_EXPECT_FLOAT_NEAR(ctx, 0.0f, handle.CurrentQController.PreviousError, 0.0f);
    TEST_EXPECT_STATE_EQ(ctx, PID_UnSaturated, handle.VoltageDController.State);
    TEST_EXPECT_STATE_EQ(ctx, PID_UnSaturated, handle.VoltageQController.State);
    TEST_EXPECT_STATE_EQ(ctx, PID_UnSaturated, handle.CurrentDController.State);
    TEST_EXPECT_STATE_EQ(ctx, PID_UnSaturated, handle.CurrentQController.State);

    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_3phOffInv_DeInit(&handle));
    TEST_EXPECT_STATE_EQ(ctx, THREEPHOFFINV_RESET, handle.State);
    VOLOOP_EXPECT_FLOAT_NEAR(ctx, 0.0f, handle.VoltageDController.KpDiscrete, 0.0f);
    VOLOOP_EXPECT_FLOAT_NEAR(ctx, 0.0f, handle.VoltageQController.KpDiscrete, 0.0f);
    VOLOOP_EXPECT_FLOAT_NEAR(ctx, 0.0f, handle.CurrentDController.KpDiscrete, 0.0f);
    VOLOOP_EXPECT_FLOAT_NEAR(ctx, 0.0f, handle.CurrentQController.KpDiscrete, 0.0f);
}

static void test_invalid_controller_init_rolls_back(VoloopTestContext* ctx) {
    ThreePhOffInv_HandleTypeDef handle = { 0 };
    NCO_InitTypeDef ncoInit = make_nco_init(0.0f);
    PID_InitTypeDef voltageDInit = make_pid_init();
    PID_InitTypeDef voltageQInit = make_pid_init();
    PID_InitTypeDef currentDInit = make_pid_init();
    PID_InitTypeDef currentQInit = make_pid_init();
    ThreePhOffInv_ConfigTypeDef config = make_config();
    ThreePhOffInv_InitTypeDef init = {
        .NCOInit = &ncoInit,
        .Config = &config,
        .VoltageDControllerInit = &voltageDInit,
        .VoltageQControllerInit = &voltageQInit,
        .CurrentDControllerInit = NULL,
        .CurrentQControllerInit = &currentQInit,
    };

    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_3phOffInv_Init(&handle, &init));
    TEST_EXPECT_STATE_EQ(ctx, THREEPHOFFINV_RESET, handle.State);

    init.CurrentDControllerInit = &currentDInit;
    init.CurrentQControllerInit = NULL;
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_3phOffInv_Init(&handle, &init));
    TEST_EXPECT_STATE_EQ(ctx, THREEPHOFFINV_RESET, handle.State);

    currentDInit.init.OneZero.triggerFrequency = 0U;
    init.CurrentQControllerInit = &currentQInit;
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_3phOffInv_Init(&handle, &init));
    TEST_EXPECT_STATE_EQ(ctx, THREEPHOFFINV_RESET, handle.State);

    currentDInit = make_pid_init();
    currentQInit.init.OneZero.triggerFrequency = 0U;
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_INVALID_PARAM, VOLOOP_3phOffInv_Init(&handle, &init));
    TEST_EXPECT_STATE_EQ(ctx, THREEPHOFFINV_RESET, handle.State);
    VOLOOP_EXPECT_FLOAT_NEAR(ctx, 0.0f, handle.VoltageDController.KpDiscrete, 0.0f);
    VOLOOP_EXPECT_FLOAT_NEAR(ctx, 0.0f, handle.VoltageQController.KpDiscrete, 0.0f);
    VOLOOP_EXPECT_FLOAT_NEAR(ctx, 0.0f, handle.CurrentDController.KpDiscrete, 0.0f);
    VOLOOP_EXPECT_FLOAT_NEAR(ctx, 0.0f, handle.CurrentQController.KpDiscrete, 0.0f);
}

static void test_line_voltage_peak_setter(VoloopTestContext* ctx) {
    ThreePhOffInv_HandleTypeDef handle = { 0 };

    TEST_EXPECT_STATUS_EQ(
        ctx, VOLOOP_INVALID_PARAM,
        VOLOOP_3phOffInv_SetLineVoltagePeak(NULL, 100.0f));
    TEST_EXPECT_STATUS_EQ(
        ctx, VOLOOP_INVALID_STATE,
        VOLOOP_3phOffInv_SetLineVoltagePeak(&handle, 100.0f));

    init_inverter(ctx, &handle);
    VOLOOP_EXPECT_FLOAT_NEAR(ctx, 0.0f, handle.TargetLineVoltagePeak, 0.0f);

    TEST_EXPECT_STATUS_EQ(
        ctx, VOLOOP_INVALID_PARAM,
        VOLOOP_3phOffInv_SetLineVoltagePeak(&handle, NAN));
    TEST_EXPECT_STATUS_EQ(
        ctx, VOLOOP_INVALID_PARAM,
        VOLOOP_3phOffInv_SetLineVoltagePeak(&handle, -1.0f));
    TEST_EXPECT_STATUS_EQ(
        ctx, VOLOOP_INVALID_PARAM,
        VOLOOP_3phOffInv_SetLineVoltagePeak(
            &handle, handle.Config.LineOverVoltageThreshold + 1.0f));
    VOLOOP_EXPECT_FLOAT_NEAR(ctx, 0.0f, handle.TargetLineVoltagePeak, 0.0f);

    TEST_REQUIRE_STATUS_EQ(
        ctx, VOLOOP_OK,
        VOLOOP_3phOffInv_SetLineVoltagePeak(&handle, 0.0f));
    TEST_REQUIRE_STATUS_EQ(
        ctx, VOLOOP_OK,
        VOLOOP_3phOffInv_SetLineVoltagePeak(
            &handle, handle.Config.LineOverVoltageThreshold));
    VOLOOP_EXPECT_FLOAT_NEAR(
        ctx, handle.Config.LineOverVoltageThreshold,
        handle.TargetLineVoltagePeak, 0.0f);

    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_3phOffInv_Start(&handle));
    TEST_REQUIRE_STATUS_EQ(
        ctx, VOLOOP_OK,
        VOLOOP_3phOffInv_SetLineVoltagePeak(&handle, 325.0f));
    VOLOOP_EXPECT_FLOAT_NEAR(ctx, 325.0f, handle.TargetLineVoltagePeak, 0.0f);

    handle.State = THREEPHOFFINV_ERROR;
    TEST_EXPECT_STATUS_EQ(
        ctx, VOLOOP_INVALID_STATE,
        VOLOOP_3phOffInv_SetLineVoltagePeak(&handle, 200.0f));
    VOLOOP_EXPECT_FLOAT_NEAR(ctx, 325.0f, handle.TargetLineVoltagePeak, 0.0f);
}

int main(void) {
    VoloopTestContext ctx = VOLOOP_TEST_CONTEXT_INIT;

    voloop_run_test(&ctx, "sync guards arguments and disabled state",
                    test_sync_guards_arguments_and_disabled_state);
    voloop_run_test(&ctx, "sync latches sample and protection faults",
                    test_sync_latches_sample_and_reconstructed_protection_faults);
    voloop_run_test(&ctx, "zero bus voltage latches undervoltage",
                    test_zero_bus_voltage_latches_undervoltage_with_zero_threshold);
    voloop_run_test(&ctx, "fault clears without restart",
                    test_fault_can_be_cleared_without_restarting_output);
    voloop_run_test(&ctx, "controller lifecycle",
                    test_controller_lifecycle);
    voloop_run_test(&ctx, "invalid controller init rolls back",
                    test_invalid_controller_init_rolls_back);
    voloop_run_test(&ctx, "line voltage peak setter",
                    test_line_voltage_peak_setter);

    return voloop_test_report(&ctx);
}
