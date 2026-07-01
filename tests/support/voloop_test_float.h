#ifndef VOLOOP_TEST_FLOAT_H
#define VOLOOP_TEST_FLOAT_H

#include "voloop_test.h"

#include <math.h>
#include <stdio.h>

#define VOLOOP_EXPECT_FLOAT_NEAR(ctx, expected, actual, tolerance)                         \
    do {                                                                                   \
        VoloopTestContext* voloop_test_ctx = (ctx);                                        \
        float voloop_expected = (float)(expected);                                         \
        float voloop_actual = (float)(actual);                                             \
        float voloop_tolerance = (float)(tolerance);                                       \
        float voloop_diff = fabsf(voloop_actual - voloop_expected);                        \
        voloop_test_ctx->assert_total++;                                                   \
        if (voloop_tolerance < 0.0f || voloop_diff > voloop_tolerance) {                   \
            voloop_test_ctx->assert_failed++;                                              \
            printf("[FAIL] %s:%d: expected %g, got %g, tolerance %g, diff %g (%s)\n",      \
                   __FILE__, __LINE__, (double)voloop_expected, (double)voloop_actual,     \
                   (double)voloop_tolerance, (double)voloop_diff, #actual);                \
        }                                                                                  \
    } while (0)

#define VOLOOP_REQUIRE_FLOAT_NEAR(ctx, expected, actual, tolerance)                        \
    do {                                                                                   \
        VoloopTestContext* voloop_test_ctx = (ctx);                                        \
        float voloop_expected = (float)(expected);                                         \
        float voloop_actual = (float)(actual);                                             \
        float voloop_tolerance = (float)(tolerance);                                       \
        float voloop_diff = fabsf(voloop_actual - voloop_expected);                        \
        voloop_test_ctx->assert_total++;                                                   \
        if (voloop_tolerance < 0.0f || voloop_diff > voloop_tolerance) {                   \
            voloop_test_ctx->assert_failed++;                                              \
            printf("[FAIL] %s:%d: required %g, got %g, tolerance %g, diff %g (%s)\n",      \
                   __FILE__, __LINE__, (double)voloop_expected, (double)voloop_actual,     \
                   (double)voloop_tolerance, (double)voloop_diff, #actual);                \
            return;                                                                        \
        }                                                                                  \
    } while (0)

#endif /* VOLOOP_TEST_FLOAT_H */
