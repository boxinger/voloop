#include "voloop_3phOffInv.h"
#include "voloop_test.h"
#include "voloop_test_float.h"

#define TEST_TOLERANCE 3.0e-5f
#define TEST_ONE_OVER_SQRT3 0.57735026918962576451f
#define TEST_PHASE_120_Q31 0x55555555U

#define TEST_EXPECT_STATUS_EQ(ctx, expected, actual)                                               \
    VOLOOP_EXPECT_EQ_INT((ctx), (expected), (actual))
#define TEST_REQUIRE_STATUS_EQ(ctx, expected, actual)                                              \
    VOLOOP_REQUIRE_EQ_INT((ctx), (expected), (actual))
#define TEST_EXPECT_STATE_EQ(ctx, expected, actual)                                                \
    VOLOOP_EXPECT_EQ_INT((ctx), (expected), (actual))
#define TEST_EXPECT_FLOAT_NEAR(ctx, expected, actual)                                              \
    VOLOOP_EXPECT_FLOAT_NEAR((ctx), (expected), (actual), TEST_TOLERANCE)

static NCO_InitTypeDef make_nco_init(float initialRad) {
    NCO_InitTypeDef init = {
        .triggerFrequency = 1000U,
        .initialFrequency = 50.0f,
        .initialRad = initialRad,
    };
    return init;
}

static PID_InitTypeDef make_pid_init(float gain, float zero) {
    PID_InitTypeDef init = {
        .mode = PID_OneZero,
        .init.OneZero = {
            .gain = gain,
            .zero = zero,
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

static ThreePhOffInv_InputTypeDef make_input_at_phase(float busVoltage,
                                                       float lineVoltagePeak,
                                                       int32_t phaseAQ31) {
    int32_t phaseBQ31 = (int32_t)((uint32_t)phaseAQ31 - TEST_PHASE_120_Q31);
    int32_t phaseCQ31 = (int32_t)((uint32_t)phaseAQ31 + TEST_PHASE_120_Q31);
    float phaseVoltagePeak = lineVoltagePeak * TEST_ONE_OVER_SQRT3;
    float phaseVoltageA = phaseVoltagePeak * VOLOOP_DEF_SIN(phaseAQ31);
    float phaseVoltageB = phaseVoltagePeak * VOLOOP_DEF_SIN(phaseBQ31);
    float phaseVoltageC = phaseVoltagePeak * VOLOOP_DEF_SIN(phaseCQ31);
    ThreePhOffInv_InputTypeDef input = {
        .BusVoltage = busVoltage,
        .LineVoltageAB = phaseVoltageA - phaseVoltageB,
        .LineVoltageBC = phaseVoltageB - phaseVoltageC,
    };
    return input;
}

static void set_phase_current_at_phase(ThreePhOffInv_InputTypeDef* input,
                                       float phaseCurrentPeak,
                                       int32_t phaseAQ31) {
    int32_t phaseCQ31 = (int32_t)((uint32_t)phaseAQ31 + TEST_PHASE_120_Q31);
    input->PhaseCurrentA = phaseCurrentPeak * VOLOOP_DEF_SIN(phaseAQ31);
    input->PhaseCurrentC = phaseCurrentPeak * VOLOOP_DEF_SIN(phaseCQ31);
}

static void init_running_with_gains(VoloopTestContext* ctx,
                                    ThreePhOffInv_HandleTypeDef* handle,
                                    float initialRad, float targetLineVoltagePeak,
                                    float voltageGain, float currentGain) {
    NCO_InitTypeDef ncoInit = make_nco_init(initialRad);
    PID_InitTypeDef voltageDInit = make_pid_init(voltageGain, 20.0f);
    PID_InitTypeDef voltageQInit = make_pid_init(voltageGain, 20.0f);
    PID_InitTypeDef currentDInit = make_pid_init(currentGain, 400.0f);
    PID_InitTypeDef currentQInit = make_pid_init(currentGain, 400.0f);
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
    TEST_REQUIRE_STATUS_EQ(
        ctx, VOLOOP_OK,
        VOLOOP_3phOffInv_SetLineVoltagePeak(handle, targetLineVoltagePeak));
    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_3phOffInv_Start(handle));
}

static void init_running(VoloopTestContext* ctx, ThreePhOffInv_HandleTypeDef* handle,
                         float initialRad, float targetLineVoltagePeak) {
    init_running_with_gains(ctx, handle, initialRad, targetLineVoltagePeak,
                            0.001f, 0.05f);
}

static void expect_output_enabled(VoloopTestContext* ctx,
                                  const ThreePhOffInv_OutputTypeDef* output) {
    TEST_EXPECT_STATE_EQ(ctx, VOLOOP_PWM_ENABLE, output->PhaseAPwmState);
    TEST_EXPECT_STATE_EQ(ctx, VOLOOP_PWM_ENABLE, output->PhaseBPwmState);
    TEST_EXPECT_STATE_EQ(ctx, VOLOOP_PWM_ENABLE, output->PhaseCPwmState);
}

static void expect_duties_in_range(VoloopTestContext* ctx,
                                   const ThreePhOffInv_OutputTypeDef* output) {
    VOLOOP_EXPECT_TRUE(ctx, output->PhaseADuty >= 0.0f && output->PhaseADuty <= 1.0f);
    VOLOOP_EXPECT_TRUE(ctx, output->PhaseBDuty >= 0.0f && output->PhaseBDuty <= 1.0f);
    VOLOOP_EXPECT_TRUE(ctx, output->PhaseCDuty >= 0.0f && output->PhaseCDuty <= 1.0f);
}

static void expect_line_duties_match_input(VoloopTestContext* ctx,
                                           const ThreePhOffInv_InputTypeDef* input,
                                           const ThreePhOffInv_OutputTypeDef* output) {
    float lineVoltageAC = input->LineVoltageAB + input->LineVoltageBC;
    TEST_EXPECT_FLOAT_NEAR(ctx, input->LineVoltageAB / input->BusVoltage,
                           output->PhaseADuty - output->PhaseBDuty);
    TEST_EXPECT_FLOAT_NEAR(ctx, input->LineVoltageBC / input->BusVoltage,
                           output->PhaseBDuty - output->PhaseCDuty);
    TEST_EXPECT_FLOAT_NEAR(ctx, -lineVoltageAC / input->BusVoltage,
                           output->PhaseCDuty - output->PhaseADuty);
}

static void test_zero_target_generates_neutral_duties(VoloopTestContext* ctx) {
    ThreePhOffInv_HandleTypeDef handle = { 0 };
    ThreePhOffInv_InputTypeDef input = {
        .BusVoltage = 400.0f,
    };
    ThreePhOffInv_OutputTypeDef output = { 0 };

    init_running(ctx, &handle, 0.0f, 0.0f);
    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_3phOffInv_Sync(&handle, &input, &output));

    expect_output_enabled(ctx, &output);
    TEST_EXPECT_FLOAT_NEAR(ctx, 0.5f, output.PhaseADuty);
    TEST_EXPECT_FLOAT_NEAR(ctx, 0.5f, output.PhaseBDuty);
    TEST_EXPECT_FLOAT_NEAR(ctx, 0.5f, output.PhaseCDuty);
    TEST_EXPECT_FLOAT_NEAR(ctx, 0.0f, handle.VoltageDController.PreviousError);
    TEST_EXPECT_FLOAT_NEAR(ctx, 0.0f, handle.VoltageQController.PreviousError);
    TEST_EXPECT_FLOAT_NEAR(ctx, 0.0f, handle.CurrentDController.PreviousError);
    TEST_EXPECT_FLOAT_NEAR(ctx, 0.0f, handle.CurrentQController.PreviousError);
    TEST_EXPECT_STATE_EQ(ctx, 0, VOLOOP_3phOffInv_GetPhaseQ31(&handle));
    VOLOOP_EXPECT_TRUE(ctx, VOLOOP_NCO_GetPhaseQ31(&handle.NCO) != 0);
}

static void test_balanced_measurement_matches_d_axis_reference(VoloopTestContext* ctx) {
    ThreePhOffInv_HandleTypeDef handle = { 0 };
    ThreePhOffInv_OutputTypeDef output = { 0 };
    const float targetLineVoltagePeak = 200.0f;

    init_running(ctx, &handle, 0.0f, targetLineVoltagePeak);
    ThreePhOffInv_InputTypeDef input = make_input_at_phase(
        400.0f, targetLineVoltagePeak, VOLOOP_3phOffInv_GetPhaseQ31(&handle));
    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_3phOffInv_Sync(&handle, &input, &output));

    TEST_EXPECT_FLOAT_NEAR(ctx, 0.5f, output.PhaseADuty);
    TEST_EXPECT_FLOAT_NEAR(ctx, 0.25f, output.PhaseBDuty);
    TEST_EXPECT_FLOAT_NEAR(ctx, 0.75f, output.PhaseCDuty);
    VOLOOP_EXPECT_FLOAT_NEAR(
        ctx, 0.0f, handle.VoltageDController.PreviousError, 1.0e-3f);
    VOLOOP_EXPECT_FLOAT_NEAR(
        ctx, 0.0f, handle.VoltageQController.PreviousError, 1.0e-3f);
    VOLOOP_EXPECT_FLOAT_NEAR(
        ctx, 0.0f, handle.CurrentDController.PreviousError, 1.0e-3f);
    VOLOOP_EXPECT_FLOAT_NEAR(
        ctx, 0.0f, handle.CurrentQController.PreviousError, 1.0e-3f);
    expect_line_duties_match_input(ctx, &input, &output);
}

static void test_minmax_common_mode_preserves_line_commands(VoloopTestContext* ctx) {
    ThreePhOffInv_HandleTypeDef handle = { 0 };
    ThreePhOffInv_OutputTypeDef output = { 0 };
    const float targetLineVoltagePeak = 200.0f;

    init_running(ctx, &handle, VOLOOP_Pi * 0.5f, targetLineVoltagePeak);
    for (unsigned int index = 0U; index < 40U; ++index) {
        ThreePhOffInv_InputTypeDef input = make_input_at_phase(
            400.0f, targetLineVoltagePeak, VOLOOP_NCO_GetPhaseQ31(&handle.NCO));
        TEST_REQUIRE_STATUS_EQ(
            ctx, VOLOOP_OK,
            VOLOOP_3phOffInv_Sync(&handle, &input, &output));
        expect_output_enabled(ctx, &output);
        expect_duties_in_range(ctx, &output);
        expect_line_duties_match_input(ctx, &input, &output);
    }
}

static void test_voltage_outer_loop_and_current_inner_loop_polarity(
    VoloopTestContext* ctx) {
    ThreePhOffInv_HandleTypeDef dHandle = { 0 };
    ThreePhOffInv_InputTypeDef zeroInput = {
        .BusVoltage = 400.0f,
    };
    ThreePhOffInv_OutputTypeDef dOutput = { 0 };

    init_running(ctx, &dHandle, 0.0f, 200.0f);
    TEST_REQUIRE_STATUS_EQ(
        ctx, VOLOOP_OK,
        VOLOOP_3phOffInv_Sync(&dHandle, &zeroInput, &dOutput));
    VOLOOP_EXPECT_TRUE(ctx, dHandle.VoltageDController.PreviousError > 0.0f);
    VOLOOP_EXPECT_TRUE(ctx, dHandle.CurrentDController.PreviousError > 0.0f);
    VOLOOP_EXPECT_TRUE(ctx, dOutput.PhaseCDuty > dOutput.PhaseBDuty);

    ThreePhOffInv_HandleTypeDef qHandle = { 0 };
    ThreePhOffInv_InputTypeDef positiveQInput = {
        .BusVoltage = 400.0f,
        .LineVoltageAB = 150.0f,
        .LineVoltageBC = 0.0f,
    };
    ThreePhOffInv_OutputTypeDef qOutput = { 0 };

    init_running(ctx, &qHandle, 0.0f, 0.0f);
    TEST_REQUIRE_STATUS_EQ(
        ctx, VOLOOP_OK,
        VOLOOP_3phOffInv_Sync(&qHandle, &positiveQInput, &qOutput));
    TEST_EXPECT_FLOAT_NEAR(ctx, qOutput.PhaseBDuty, qOutput.PhaseCDuty);
    VOLOOP_EXPECT_TRUE(ctx, qHandle.VoltageQController.PreviousError < 0.0f);
    VOLOOP_EXPECT_TRUE(ctx, qHandle.CurrentQController.PreviousError < 0.0f);
}

static void test_ia_ic_reconstruction_and_current_dq_polarity(
    VoloopTestContext* ctx) {
    ThreePhOffInv_HandleTypeDef dHandle = { 0 };
    ThreePhOffInv_InputTypeDef dInput = {
        .BusVoltage = 400.0f,
    };
    ThreePhOffInv_OutputTypeDef dOutput = { 0 };

    init_running(ctx, &dHandle, 0.0f, 0.0f);
    set_phase_current_at_phase(
        &dInput, 5.0f, VOLOOP_3phOffInv_GetPhaseQ31(&dHandle));
    TEST_REQUIRE_STATUS_EQ(
        ctx, VOLOOP_OK,
        VOLOOP_3phOffInv_Sync(&dHandle, &dInput, &dOutput));
    VOLOOP_EXPECT_FLOAT_NEAR(
        ctx, -5.0f, dHandle.CurrentDController.PreviousError, 1.0e-3f);
    VOLOOP_EXPECT_FLOAT_NEAR(
        ctx, 0.0f, dHandle.CurrentQController.PreviousError, 1.0e-3f);

    ThreePhOffInv_HandleTypeDef qHandle = { 0 };
    ThreePhOffInv_InputTypeDef qInput = {
        .BusVoltage = 400.0f,
        .PhaseCurrentA = 5.0f,
        .PhaseCurrentC = -2.5f,
    };
    ThreePhOffInv_OutputTypeDef qOutput = { 0 };

    init_running(ctx, &qHandle, 0.0f, 0.0f);
    TEST_REQUIRE_STATUS_EQ(
        ctx, VOLOOP_OK,
        VOLOOP_3phOffInv_Sync(&qHandle, &qInput, &qOutput));
    VOLOOP_EXPECT_FLOAT_NEAR(
        ctx, 0.0f, qHandle.CurrentDController.PreviousError, 1.0e-3f);
    VOLOOP_EXPECT_FLOAT_NEAR(
        ctx, -5.0f, qHandle.CurrentQController.PreviousError, 1.0e-3f);
}

static void test_d_axis_priority_limits_q_and_duties(VoloopTestContext* ctx) {
    ThreePhOffInv_HandleTypeDef handle = { 0 };
    ThreePhOffInv_InputTypeDef input = {
        .BusVoltage = 400.0f,
        .LineVoltageAB = 150.0f,
        .LineVoltageBC = 0.0f,
    };
    ThreePhOffInv_OutputTypeDef output = { 0 };

    init_running_with_gains(ctx, &handle, 0.0f, 500.0f, 1.0f, 1.0f);
    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_3phOffInv_Sync(&handle, &input, &output));

    expect_duties_in_range(ctx, &output);
    TEST_EXPECT_FLOAT_NEAR(ctx, 0.5f, output.PhaseADuty);
    TEST_EXPECT_FLOAT_NEAR(ctx, 0.0f, output.PhaseBDuty);
    TEST_EXPECT_FLOAT_NEAR(ctx, 1.0f, output.PhaseCDuty);
    TEST_EXPECT_STATE_EQ(ctx, PID_UpperSaturated, handle.VoltageDController.State);
    TEST_EXPECT_STATE_EQ(ctx, PID_LowerSaturated, handle.VoltageQController.State);
    TEST_EXPECT_STATE_EQ(ctx, PID_UpperSaturated, handle.CurrentDController.State);
}

static void test_frequency_update_and_restart_phase_contract(VoloopTestContext* ctx) {
    ThreePhOffInv_HandleTypeDef handle = { 0 };
    ThreePhOffInv_InputTypeDef input = {
        .BusVoltage = 400.0f,
    };
    ThreePhOffInv_OutputTypeDef output = { 0 };

    init_running(ctx, &handle, 0.0f, 0.0f);
    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_3phOffInv_SetFrequency(&handle, 100.0f));
    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_3phOffInv_Sync(&handle, &input, &output));
    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_3phOffInv_Sync(&handle, &input, &output));
    TEST_EXPECT_FLOAT_NEAR(ctx, 0.2f * VOLOOP_Pi,
                           VOLOOP_DEF_Q31ToRad(VOLOOP_3phOffInv_GetPhaseQ31(&handle)));

    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_3phOffInv_Stop(&handle));
    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_3phOffInv_Start(&handle));
    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_3phOffInv_Sync(&handle, &input, &output));
    TEST_EXPECT_STATE_EQ(ctx, 0, VOLOOP_3phOffInv_GetPhaseQ31(&handle));
    TEST_EXPECT_FLOAT_NEAR(ctx, 0.5f, output.PhaseADuty);
}

static void test_threshold_equality_remains_operational(VoloopTestContext* ctx) {
    ThreePhOffInv_HandleTypeDef handle = { 0 };
    ThreePhOffInv_InputTypeDef input = {
        .BusVoltage = 450.0f,
        .LineVoltageAB = 500.0f,
        .LineVoltageBC = 0.0f,
        .PhaseCurrentA = 20.0f,
        .PhaseCurrentC = -20.0f,
    };
    ThreePhOffInv_OutputTypeDef output = { 0 };

    init_running(ctx, &handle, 0.0f, 0.0f);
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_3phOffInv_Sync(&handle, &input, &output));
    TEST_EXPECT_STATE_EQ(ctx, THREEPHOFFINV_RUNNING, VOLOOP_3phOffInv_GetState(&handle));
    TEST_EXPECT_STATE_EQ(ctx, THREEPHOFFINV_FAULT_NONE, VOLOOP_3phOffInv_GetFaultCode(&handle));
    expect_output_enabled(ctx, &output);
    expect_duties_in_range(ctx, &output);
}

int main(void) {
    VoloopTestContext ctx = VOLOOP_TEST_CONTEXT_INIT;

    voloop_run_test(&ctx, "zero target generates neutral duties",
                    test_zero_target_generates_neutral_duties);
    voloop_run_test(&ctx, "balanced measurement matches d-axis reference",
                    test_balanced_measurement_matches_d_axis_reference);
    voloop_run_test(&ctx, "min-max common mode preserves line commands",
                    test_minmax_common_mode_preserves_line_commands);
    voloop_run_test(&ctx, "voltage/current loop polarity",
                    test_voltage_outer_loop_and_current_inner_loop_polarity);
    voloop_run_test(&ctx, "Ia/Ic reconstruction and current d/q polarity",
                    test_ia_ic_reconstruction_and_current_dq_polarity);
    voloop_run_test(&ctx, "d-axis priority limits q and duties",
                    test_d_axis_priority_limits_q_and_duties);
    voloop_run_test(&ctx, "frequency update and restart phase",
                    test_frequency_update_and_restart_phase_contract);
    voloop_run_test(&ctx, "threshold equality remains operational",
                    test_threshold_equality_remains_operational);

    return voloop_test_report(&ctx);
}
