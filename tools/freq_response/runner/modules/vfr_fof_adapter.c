#include "vfr_fof_adapter.h"

#include <stddef.h>

static float vfr_fof_compute(void* context, float input) {
    VFR_FofSubject* subject = (VFR_FofSubject*)context;

    if (subject == NULL) {
        return 0.0f;
    }

    return VOLOOP_FOF_Compute(&subject->handle, input);
}

static void vfr_fof_reset(void* context) {
    VFR_FofSubject* subject = (VFR_FofSubject*)context;

    if (subject == NULL) {
        return;
    }

    (void)VOLOOP_FOF_Reset(&subject->handle);
}

static void vfr_set_fof_test_subject(VFR_FofSubject* fof_subject,
                                     VFR_TestSubject* test_subject) {
    test_subject->context = fof_subject;
    test_subject->compute = vfr_fof_compute;
    test_subject->reset = vfr_fof_reset;
}

int VFR_InitFofDiscreteSubject(VFR_FofSubject* fof_subject,
                               VFR_TestSubject* test_subject,
                               float b0,
                               float b1,
                               float a1) {
    FOF_InitTypeDef init = {0};

    if (fof_subject == NULL || test_subject == NULL) {
        return 0;
    }

    init.mode = FOF_Discrete;
    init.init.Discrete.b0 = b0;
    init.init.Discrete.b1 = b1;
    init.init.Discrete.a1 = a1;

    if (VOLOOP_FOF_Init(&fof_subject->handle, &init) != VOLOOP_OK) {
        return 0;
    }

    vfr_set_fof_test_subject(fof_subject, test_subject);

    return 1;
}

int VFR_InitFofLowPassSubject(VFR_FofSubject* fof_subject,
                              VFR_TestSubject* test_subject,
                              float cutoff_hz,
                              float trigger_frequency_hz) {
    FOF_InitTypeDef init = {0};

    if (fof_subject == NULL || test_subject == NULL) {
        return 0;
    }

    init.mode = FOF_LowPass;
    init.init.LowPass.cutoffFrequency = cutoff_hz;
    init.init.LowPass.triggerFrequency = trigger_frequency_hz;

    if (VOLOOP_FOF_Init(&fof_subject->handle, &init) != VOLOOP_OK) {
        return 0;
    }

    vfr_set_fof_test_subject(fof_subject, test_subject);

    return 1;
}

int VFR_InitFofHighPassSubject(VFR_FofSubject* fof_subject,
                               VFR_TestSubject* test_subject,
                               float cutoff_hz,
                               float trigger_frequency_hz) {
    FOF_InitTypeDef init = {0};

    if (fof_subject == NULL || test_subject == NULL) {
        return 0;
    }

    init.mode = FOF_HighPass;
    init.init.HighPass.cutoffFrequency = cutoff_hz;
    init.init.HighPass.triggerFrequency = trigger_frequency_hz;

    if (VOLOOP_FOF_Init(&fof_subject->handle, &init) != VOLOOP_OK) {
        return 0;
    }

    vfr_set_fof_test_subject(fof_subject, test_subject);

    return 1;
}

int VFR_InitFofLeadLagSubject(VFR_FofSubject* fof_subject,
                              VFR_TestSubject* test_subject,
                              float zero_hz,
                              float pole_hz,
                              float gain,
                              float trigger_frequency_hz) {
    FOF_InitTypeDef init = {0};

    if (fof_subject == NULL || test_subject == NULL) {
        return 0;
    }

    init.mode = FOF_LeadLag;
    init.init.LeadLag.zero = zero_hz;
    init.init.LeadLag.pole = pole_hz;
    init.init.LeadLag.gain = gain;
    init.init.LeadLag.triggerFrequency = trigger_frequency_hz;

    if (VOLOOP_FOF_Init(&fof_subject->handle, &init) != VOLOOP_OK) {
        return 0;
    }

    vfr_set_fof_test_subject(fof_subject, test_subject);

    return 1;
}

int VFR_InitFofContinueSubject(VFR_FofSubject* fof_subject,
                               VFR_TestSubject* test_subject,
                               float K,
                               float b0,
                               float b1,
                               float a0,
                               float a1,
                               float trigger_frequency_hz) {
    FOF_InitTypeDef init = {0};

    if (fof_subject == NULL || test_subject == NULL) {
        return 0;
    }

    init.mode = FOF_Continue;
    init.init.Continue.K = K;
    init.init.Continue.b0 = b0;
    init.init.Continue.b1 = b1;
    init.init.Continue.a0 = a0;
    init.init.Continue.a1 = a1;
    init.init.Continue.triggerFrequency = trigger_frequency_hz;

    if (VOLOOP_FOF_Init(&fof_subject->handle, &init) != VOLOOP_OK) {
        return 0;
    }

    vfr_set_fof_test_subject(fof_subject, test_subject);

    return 1;
}
