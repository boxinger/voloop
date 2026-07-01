#ifndef VOLOOP_TEST_FLOAT_H
#define VOLOOP_TEST_FLOAT_H

#include <math.h>
#include <stdio.h>

#define TEST_ASSERT_FLOAT_NEAR(expected, actual, tolerance)                            \
    do {                                                                               \
        float expected_value = (float)(expected);                                      \
        float actual_value = (float)(actual);                                          \
        float tolerance_value = (float)(tolerance);                                    \
        if (fabsf(actual_value - expected_value) > tolerance_value) {                  \
            printf("%s:%d: expected %s near %g, got %g, tolerance %g\n", __FILE__,     \
                   __LINE__, #actual, (double)expected_value, (double)actual_value,    \
                   (double)tolerance_value);                                           \
            return 1;                                                                  \
        }                                                                              \
    } while (0)

#endif /* VOLOOP_TEST_FLOAT_H */
