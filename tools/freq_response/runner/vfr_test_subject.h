#ifndef VFR_TEST_SUBJECT_H
#define VFR_TEST_SUBJECT_H

/**
 * @file vfr_test_subject.h
 * @brief Minimal black-box subject interface for VOLOOP Frequency Response tools.
 */

typedef float (*VFR_ComputeSampleFn)(void* context, float input);
typedef void (*VFR_ResetFn)(void* context);

/**
 * @brief Black-box single-input single-output test subject.
 *
 * The measurement code treats the subject as an opaque input -> output
 * transform. The context pointer is passed to both callbacks and is owned by
 * the caller.
 */
typedef struct {
    void* context;              /**< User-owned state passed to callback functions. */
    VFR_ComputeSampleFn compute; /**< Compute one output sample from one input sample. */
    VFR_ResetFn reset;           /**< Optional reset callback called before each frequency point. */
} VFR_TestSubject;

#endif /* VFR_TEST_SUBJECT_H */
