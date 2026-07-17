#include "vfr_point_measure.h"

#include <math.h>
#include <stdio.h>

typedef struct {
    unsigned int failures;
} VFR_TestContext;

static float unity_compute(void* context, float input) {
    (void)context;
    return input;
}

static float gain2_compute(void* context, float input) {
    (void)context;
    return 2.0f * input;
}

static float negative_unity_compute(void* context, float input) {
    (void)context;
    return -input;
}

static float zero_compute(void* context, float input) {
    (void)context;
    (void)input;
    return 0.0f;
}

static float exploding_compute(void* context, float input) {
    (void)context;
    (void)input;
    return 100.0f;
}

static VFR_PointMeasureConfig make_default_config(void) {
    VFR_PointMeasureConfig config = {0};

    config.sample_rate_hz = 10000.0;
    config.frequency_hz = 50.0;
    config.input_amplitude = 1.0;
    config.warmup_cycles = 2U;
    config.measure_cycles = 8U;
    config.min_samples_per_cycle = 20U;
    config.max_samples_per_point = 100000U;
    config.output_abs_limit = 10.0;
    config.gain_floor = 1.0e-12;

    return config;
}

static void expect_status(VFR_TestContext* ctx,
                          const char* name,
                          VFR_PointMeasureStatus expected,
                          VFR_PointMeasureStatus actual) {
    if (actual != expected) {
        ++ctx->failures;
        printf("[FAIL] %s: expected status %s, got %s\n", name,
               VFR_PointMeasureStatusToString(expected), VFR_PointMeasureStatusToString(actual));
    }
}

static void expect_near(VFR_TestContext* ctx,
                        const char* name,
                        const char* field,
                        double expected,
                        double actual,
                        double tolerance) {
    double diff = fabs(actual - expected);
    if (!isfinite(actual) || diff > tolerance) {
        ++ctx->failures;
        printf("[FAIL] %s: expected %s %.8f, got %.8f, tolerance %.8f\n", name, field,
               expected, actual, tolerance);
    }
}

static void test_unity(VFR_TestContext* ctx) {
    const char* name = "unity";
    VFR_PointMeasureConfig config = make_default_config();
    VFR_TestSubject subject = {NULL, unity_compute, NULL};
    VFR_PointMeasureResult result = {0};
    VFR_PointMeasureStatus status = VFR_MeasurePoint(&subject, &config, &result);

    expect_status(ctx, name, VFR_POINT_OK, status);
    expect_near(ctx, name, "gain_db", 0.0, result.gain_db, 0.01);
    expect_near(ctx, name, "phase_deg", 0.0, result.phase_deg, 1.0);
}

static void test_gain2(VFR_TestContext* ctx) {
    const char* name = "gain2";
    VFR_PointMeasureConfig config = make_default_config();
    VFR_TestSubject subject = {NULL, gain2_compute, NULL};
    VFR_PointMeasureResult result = {0};
    VFR_PointMeasureStatus status = VFR_MeasurePoint(&subject, &config, &result);

    expect_status(ctx, name, VFR_POINT_OK, status);
    expect_near(ctx, name, "gain_db", 6.0206, result.gain_db, 0.01);
    expect_near(ctx, name, "phase_deg", 0.0, result.phase_deg, 1.0);
}

static void test_negative_unity(VFR_TestContext* ctx) {
    const char* name = "negative_unity";
    VFR_PointMeasureConfig config = make_default_config();
    VFR_TestSubject subject = {NULL, negative_unity_compute, NULL};
    VFR_PointMeasureResult result = {0};
    VFR_PointMeasureStatus status = VFR_MeasurePoint(&subject, &config, &result);
    double phase_error = fabs(fabs(result.phase_deg) - 180.0);

    expect_status(ctx, name, VFR_POINT_OK, status);
    expect_near(ctx, name, "gain_db", 0.0, result.gain_db, 0.01);
    if (!isfinite(result.phase_deg) || phase_error > 1.0) {
        ++ctx->failures;
        printf("[FAIL] %s: expected phase near +/-180 deg, got %.8f\n", name, result.phase_deg);
    }
}

static void test_zero(VFR_TestContext* ctx) {
    const char* name = "zero";
    VFR_PointMeasureConfig config = make_default_config();
    VFR_TestSubject subject = {NULL, zero_compute, NULL};
    VFR_PointMeasureResult result = {0};
    VFR_PointMeasureStatus status = VFR_MeasurePoint(&subject, &config, &result);

    expect_status(ctx, name, VFR_POINT_OK_WITH_GAIN_FLOOR, status);
}

static void test_exploding(VFR_TestContext* ctx) {
    const char* name = "exploding";
    VFR_PointMeasureConfig config = make_default_config();
    VFR_TestSubject subject = {NULL, exploding_compute, NULL};
    VFR_PointMeasureResult result = {0};
    VFR_PointMeasureStatus status = VFR_MeasurePoint(&subject, &config, &result);

    expect_status(ctx, name, VFR_POINT_OUTPUT_LIMIT_EXCEEDED, status);
}

static void test_measure_sample_limit(VFR_TestContext* ctx) {
    const char* name = "measure_sample_limit";
    VFR_PointMeasureConfig config = make_default_config();
    VFR_TestSubject subject = {NULL, unity_compute, NULL};
    VFR_PointMeasureResult result = {0};
    VFR_PointMeasureStatus status;

    config.max_samples_per_point = 1599U;
    status = VFR_MeasurePoint(&subject, &config, &result);

    expect_status(ctx, name, VFR_POINT_SAMPLE_LIMIT_EXCEEDED, status);
}

static void test_warmup_does_not_count_against_sample_limit(VFR_TestContext* ctx) {
    const char* name = "warmup_does_not_count_against_sample_limit";
    VFR_PointMeasureConfig config = make_default_config();
    VFR_TestSubject subject = {NULL, unity_compute, NULL};
    VFR_PointMeasureResult result = {0};
    VFR_PointMeasureStatus status;

    config.max_samples_per_point = 1600U;
    status = VFR_MeasurePoint(&subject, &config, &result);

    expect_status(ctx, name, VFR_POINT_OK, status);
}

int main(void) {
    VFR_TestContext ctx = {0U};

    test_unity(&ctx);
    test_gain2(&ctx);
    test_negative_unity(&ctx);
    test_zero(&ctx);
    test_exploding(&ctx);
    test_measure_sample_limit(&ctx);
    test_warmup_does_not_count_against_sample_limit(&ctx);

    if (ctx.failures != 0U) {
        printf("[FAIL] vfr_point_measure_test: %u failure(s)\n", ctx.failures);
        return 1;
    }

    printf("[PASS] vfr_point_measure_test\n");
    return 0;
}
