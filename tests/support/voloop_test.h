#ifndef VOLOOP_TEST_H
#define VOLOOP_TEST_H

#include <stdio.h>

typedef struct {
    unsigned int assert_total;
    unsigned int assert_failed;
    unsigned int case_total;
    unsigned int case_failed;
} VoloopTestContext;

#define VOLOOP_TEST_CONTEXT_INIT {0U, 0U, 0U, 0U}

typedef void (*VoloopTestFunction)(VoloopTestContext* ctx);

#define VOLOOP_EXPECT_TRUE(ctx, condition)                                                \
    do {                                                                                  \
        VoloopTestContext* voloop_test_ctx = (ctx);                                       \
        voloop_test_ctx->assert_total++;                                                  \
        if (!(condition)) {                                                               \
            voloop_test_ctx->assert_failed++;                                             \
            printf("[FAIL] %s:%d: expected true: %s\n", __FILE__, __LINE__, #condition); \
        }                                                                                 \
    } while (0)

#define VOLOOP_REQUIRE_TRUE(ctx, condition)                                               \
    do {                                                                                  \
        VoloopTestContext* voloop_test_ctx = (ctx);                                       \
        voloop_test_ctx->assert_total++;                                                  \
        if (!(condition)) {                                                               \
            voloop_test_ctx->assert_failed++;                                             \
            printf("[FAIL] %s:%d: required true: %s\n", __FILE__, __LINE__, #condition); \
            return;                                                                       \
        }                                                                                 \
    } while (0)

#define VOLOOP_EXPECT_EQ_INT(ctx, expected, actual)                                      \
    do {                                                                                 \
        VoloopTestContext* voloop_test_ctx = (ctx);                                      \
        int voloop_expected = (int)(expected);                                           \
        int voloop_actual = (int)(actual);                                               \
        voloop_test_ctx->assert_total++;                                                 \
        if (voloop_actual != voloop_expected) {                                          \
            voloop_test_ctx->assert_failed++;                                            \
            printf("[FAIL] %s:%d: expected %d, got %d (%s)\n", __FILE__, __LINE__,      \
                   voloop_expected, voloop_actual, #actual);                            \
        }                                                                                \
    } while (0)

#define VOLOOP_REQUIRE_EQ_INT(ctx, expected, actual)                                     \
    do {                                                                                 \
        VoloopTestContext* voloop_test_ctx = (ctx);                                      \
        int voloop_expected = (int)(expected);                                           \
        int voloop_actual = (int)(actual);                                               \
        voloop_test_ctx->assert_total++;                                                 \
        if (voloop_actual != voloop_expected) {                                          \
            voloop_test_ctx->assert_failed++;                                            \
            printf("[FAIL] %s:%d: required %d, got %d (%s)\n", __FILE__, __LINE__,      \
                   voloop_expected, voloop_actual, #actual);                            \
            return;                                                                      \
        }                                                                                \
    } while (0)

static void voloop_run_test(VoloopTestContext* ctx, const char* name, VoloopTestFunction test) {
    unsigned int failed_before = ctx->assert_failed;

    ctx->case_total++;
    printf("[RUN ] %s\n", name);

    test(ctx);

    if (ctx->assert_failed != failed_before) {
        ctx->case_failed++;
        printf("[FAIL] %s\n", name);
    } else {
        printf("[PASS] %s\n", name);
    }
}

static int voloop_test_report(const VoloopTestContext* ctx) {
    printf("\n[TEST SUMMARY]\n");
    printf("  cases total   : %u\n", ctx->case_total);
    printf("  cases failed  : %u\n", ctx->case_failed);
    printf("  asserts total : %u\n", ctx->assert_total);
    printf("  asserts failed: %u\n", ctx->assert_failed);

    if (ctx->assert_failed == 0U) {
        return 0;
    }
    if (ctx->assert_failed > 255U) {
        return 255;
    }
    return (int)ctx->assert_failed;
}

#endif /* VOLOOP_TEST_H */
