#ifndef VOLOOP_TEST_H
#define VOLOOP_TEST_H

#include <stdio.h>

typedef int (*VolLoopTestFunction)(void);

#define TEST_ASSERT_TRUE(condition)                                                   \
    do {                                                                              \
        if (!(condition)) {                                                           \
            printf("%s:%d: assertion failed: %s\n", __FILE__, __LINE__, #condition); \
            return 1;                                                                 \
        }                                                                             \
    } while (0)

#define TEST_ASSERT_INT_EQ(expected, actual)                                           \
    do {                                                                               \
        int expected_value = (int)(expected);                                          \
        int actual_value = (int)(actual);                                              \
        if (actual_value != expected_value) {                                          \
            printf("%s:%d: expected %s == %d, got %d\n", __FILE__, __LINE__, #actual, \
                   expected_value, actual_value);                                      \
            return 1;                                                                  \
        }                                                                              \
    } while (0)

static int voloop_run_test(const char* name, VolLoopTestFunction test) {
    if (test() != 0) {
        printf("FAILED: %s\n", name);
        return 1;
    }

    return 0;
}

#endif /* VOLOOP_TEST_H */
