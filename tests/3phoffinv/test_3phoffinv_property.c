#include "voloop_3phOffInv.h"
#include "voloop_test.h"
#include "voloop_test_float.h"

#define TEST_TOLERANCE 2.0e-5f
#define TEST_PHASE_120_Q31 0x55555555U
#define TEST_DUTY_SCALE 0.25f

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
    };
    return input;
}

static void init_running(VoloopTestContext* ctx, ThreePhOffInv_HandleTypeDef* handle,
                         float initialRad) {
    NCO_InitTypeDef ncoInit = make_nco_init(initialRad);
    ThreePhOffInv_ConfigTypeDef config = make_config();
    ThreePhOffInv_InitTypeDef init = {
        .NCOInit = &ncoInit,
        .Config = &config,
    };

    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_3phOffInv_Init(handle, &init));
    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_3phOffInv_Start(handle));
}

static void expect_output_enabled(VoloopTestContext* ctx,
                                  const ThreePhOffInv_OutputTypeDef* output) {
    TEST_EXPECT_STATE_EQ(ctx, VOLOOP_PWM_ENABLE, output->PhaseAPwmState);
    TEST_EXPECT_STATE_EQ(ctx, VOLOOP_PWM_ENABLE, output->PhaseBPwmState);
    TEST_EXPECT_STATE_EQ(ctx, VOLOOP_PWM_ENABLE, output->PhaseCPwmState);
}

static void expect_line_duties_match_sinusoidal_references(
    VoloopTestContext* ctx, const ThreePhOffInv_HandleTypeDef* handle,
    const ThreePhOffInv_OutputTypeDef* output) {
    int32_t phaseAQ31 = VOLOOP_3phOffInv_GetPhaseQ31(handle);
    int32_t phaseBQ31 = (int32_t)((uint32_t)phaseAQ31 - TEST_PHASE_120_Q31);
    int32_t phaseCQ31 = (int32_t)((uint32_t)phaseAQ31 + TEST_PHASE_120_Q31);
    float referenceA = VOLOOP_DEF_SIN(phaseAQ31);
    float referenceB = VOLOOP_DEF_SIN(phaseBQ31);
    float referenceC = VOLOOP_DEF_SIN(phaseCQ31);

    TEST_EXPECT_FLOAT_NEAR(ctx, TEST_DUTY_SCALE * (referenceA - referenceB),
                           output->PhaseADuty - output->PhaseBDuty);
    TEST_EXPECT_FLOAT_NEAR(ctx, TEST_DUTY_SCALE * (referenceB - referenceC),
                           output->PhaseBDuty - output->PhaseCDuty);
    TEST_EXPECT_FLOAT_NEAR(ctx, TEST_DUTY_SCALE * (referenceC - referenceA),
                           output->PhaseCDuty - output->PhaseADuty);
}

static void test_zero_phase_generates_positive_sequence_duties(VoloopTestContext* ctx) {
    ThreePhOffInv_HandleTypeDef handle = { 0 };
    ThreePhOffInv_InputTypeDef input = make_valid_input();
    ThreePhOffInv_OutputTypeDef output = { 0 };

    init_running(ctx, &handle, 0.0f);
    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_3phOffInv_Sync(&handle, &input, &output));

    expect_output_enabled(ctx, &output);
    TEST_EXPECT_FLOAT_NEAR(ctx, 0.5f, output.PhaseADuty);
    TEST_EXPECT_FLOAT_NEAR(ctx, 0.28349365f, output.PhaseBDuty);
    TEST_EXPECT_FLOAT_NEAR(ctx, 0.71650635f, output.PhaseCDuty);
    TEST_EXPECT_FLOAT_NEAR(ctx, 1.5f, output.PhaseADuty + output.PhaseBDuty + output.PhaseCDuty);
    expect_line_duties_match_sinusoidal_references(ctx, &handle, &output);
    TEST_EXPECT_STATE_EQ(ctx, 0, VOLOOP_3phOffInv_GetPhaseQ31(&handle));
    VOLOOP_EXPECT_TRUE(ctx, VOLOOP_NCO_GetPhaseQ31(&handle.NCO) != 0);
}

static void test_thipwm_quarter_cycle_duties_and_bounds(VoloopTestContext* ctx) {
    ThreePhOffInv_HandleTypeDef handle = { 0 };
    ThreePhOffInv_InputTypeDef input = make_valid_input();
    ThreePhOffInv_OutputTypeDef output = { 0 };

    init_running(ctx, &handle, VOLOOP_Pi * 0.5f);
    TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_3phOffInv_Sync(&handle, &input, &output));

    TEST_EXPECT_FLOAT_NEAR(ctx, 0.70833333f, output.PhaseADuty);
    TEST_EXPECT_FLOAT_NEAR(ctx, 0.33333333f, output.PhaseBDuty);
    TEST_EXPECT_FLOAT_NEAR(ctx, 0.33333333f, output.PhaseCDuty);
    expect_line_duties_match_sinusoidal_references(ctx, &handle, &output);

    for (unsigned int index = 0U; index < 40U; ++index) {
        TEST_REQUIRE_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_3phOffInv_Sync(&handle, &input, &output));
        VOLOOP_EXPECT_TRUE(ctx, output.PhaseADuty >= 0.2834f && output.PhaseADuty <= 0.7166f);
        VOLOOP_EXPECT_TRUE(ctx, output.PhaseBDuty >= 0.2834f && output.PhaseBDuty <= 0.7166f);
        VOLOOP_EXPECT_TRUE(ctx, output.PhaseCDuty >= 0.2834f && output.PhaseCDuty <= 0.7166f);
        expect_line_duties_match_sinusoidal_references(ctx, &handle, &output);
    }
}

static void test_frequency_update_and_restart_phase_contract(VoloopTestContext* ctx) {
    ThreePhOffInv_HandleTypeDef handle = { 0 };
    ThreePhOffInv_InputTypeDef input = make_valid_input();
    ThreePhOffInv_OutputTypeDef output = { 0 };

    init_running(ctx, &handle, 0.0f);
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
        .LineVoltageAC = 0.0f,
        .PhaseCurrentA = 20.0f,
        .PhaseCurrentB = -20.0f,
    };
    ThreePhOffInv_OutputTypeDef output = { 0 };

    init_running(ctx, &handle, 0.0f);
    TEST_EXPECT_STATUS_EQ(ctx, VOLOOP_OK, VOLOOP_3phOffInv_Sync(&handle, &input, &output));
    TEST_EXPECT_STATE_EQ(ctx, THREEPHOFFINV_RUNNING, VOLOOP_3phOffInv_GetState(&handle));
    TEST_EXPECT_STATE_EQ(ctx, THREEPHOFFINV_FAULT_NONE, VOLOOP_3phOffInv_GetFaultCode(&handle));
    expect_output_enabled(ctx, &output);
}

int main(void) {
    VoloopTestContext ctx = VOLOOP_TEST_CONTEXT_INIT;

    voloop_run_test(&ctx, "zero phase generates positive sequence",
                    test_zero_phase_generates_positive_sequence_duties);
    voloop_run_test(&ctx, "THIPWM quarter cycle and line duties",
                    test_thipwm_quarter_cycle_duties_and_bounds);
    voloop_run_test(&ctx, "frequency update and restart phase",
                    test_frequency_update_and_restart_phase_contract);
    voloop_run_test(&ctx, "threshold equality remains operational",
                    test_threshold_equality_remains_operational);

    return voloop_test_report(&ctx);
}
