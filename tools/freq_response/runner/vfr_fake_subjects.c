#include "vfr_fake_subjects.h"

#include <stddef.h>
#include <string.h>

static float vfr_unity_compute(void* context, float input) {
    (void)context;
    return input;
}

static float vfr_gain2_compute(void* context, float input) {
    (void)context;
    return 2.0f * input;
}

static float vfr_negative_unity_compute(void* context, float input) {
    (void)context;
    return -input;
}

static float vfr_zero_compute(void* context, float input) {
    (void)context;
    (void)input;
    return 0.0f;
}

static float vfr_exploding_compute(void* context, float input) {
    (void)context;
    (void)input;
    return 1.0e30f;
}

static void vfr_set_subject(VFR_TestSubject* subject, VFR_ComputeSampleFn compute) {
    subject->context = NULL;
    subject->compute = compute;
    subject->reset = NULL;
}

int VFR_GetFakeSubject(const char* name, VFR_TestSubject* subject) {
    if (name == NULL || subject == NULL) {
        return 0;
    }

    if (strcmp(name, "unity") == 0) {
        vfr_set_subject(subject, vfr_unity_compute);
        return 1;
    }
    if (strcmp(name, "gain2") == 0) {
        vfr_set_subject(subject, vfr_gain2_compute);
        return 1;
    }
    if (strcmp(name, "negative_unity") == 0) {
        vfr_set_subject(subject, vfr_negative_unity_compute);
        return 1;
    }
    if (strcmp(name, "zero") == 0) {
        vfr_set_subject(subject, vfr_zero_compute);
        return 1;
    }
    if (strcmp(name, "exploding") == 0) {
        vfr_set_subject(subject, vfr_exploding_compute);
        return 1;
    }

    return 0;
}
