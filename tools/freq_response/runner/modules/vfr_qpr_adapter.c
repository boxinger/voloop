#include "vfr_qpr_adapter.h"

#include <stddef.h>

static float vfr_qpr_compute(void* context, float input) {
    VFR_QprSubject* subject = (VFR_QprSubject*)context;

    if (subject == NULL) {
        return 0.0f;
    }

    return VOLOOP_QPR_Compute(&subject->handle, input);
}

static void vfr_qpr_reset(void* context) {
    VFR_QprSubject* subject = (VFR_QprSubject*)context;

    if (subject == NULL) {
        return;
    }

    (void)VOLOOP_QPR_Reset(&subject->handle);
}

static void vfr_set_qpr_test_subject(VFR_QprSubject* qpr_subject,
                                     VFR_TestSubject* test_subject) {
    test_subject->context = qpr_subject;
    test_subject->compute = vfr_qpr_compute;
    test_subject->reset = vfr_qpr_reset;
}

int VFR_InitQprDiscreteSubject(VFR_QprSubject* qpr_subject,
                               VFR_TestSubject* test_subject,
                               float b0,
                               float b1,
                               float b2,
                               float a1,
                               float a2) {
    QPR_InitTypeDef init = {0};

    if (qpr_subject == NULL || test_subject == NULL) {
        return 0;
    }

    init.mode = QPR_Discrete;
    init.init.Discrete.b0 = b0;
    init.init.Discrete.b1 = b1;
    init.init.Discrete.b2 = b2;
    init.init.Discrete.a1 = a1;
    init.init.Discrete.a2 = a2;

    if (VOLOOP_QPR_Init(&qpr_subject->handle, &init) != VOLOOP_OK) {
        return 0;
    }

    vfr_set_qpr_test_subject(qpr_subject, test_subject);

    return 1;
}

int VFR_InitQprIdealSubject(VFR_QprSubject* qpr_subject,
                            VFR_TestSubject* test_subject,
                            float kp,
                            float kr,
                            float resonant_frequency_hz,
                            float trigger_frequency_hz) {
    QPR_InitTypeDef init = {0};

    if (qpr_subject == NULL || test_subject == NULL) {
        return 0;
    }

    init.mode = QPR_Ideal;
    init.init.Ideal.Kp = kp;
    init.init.Ideal.Kr = kr;
    init.init.Ideal.resonantFrequency = resonant_frequency_hz;
    init.init.Ideal.triggerFrequency = trigger_frequency_hz;

    if (VOLOOP_QPR_Init(&qpr_subject->handle, &init) != VOLOOP_OK) {
        return 0;
    }

    vfr_set_qpr_test_subject(qpr_subject, test_subject);

    return 1;
}

int VFR_InitQprNonIdealSubject(VFR_QprSubject* qpr_subject,
                               VFR_TestSubject* test_subject,
                               float kp,
                               float kr,
                               float resonant_frequency_hz,
                               float cutoff_frequency_hz,
                               float trigger_frequency_hz) {
    QPR_InitTypeDef init = {0};

    if (qpr_subject == NULL || test_subject == NULL) {
        return 0;
    }

    init.mode = QPR_NonIdeal;
    init.init.NonIdeal.Kp = kp;
    init.init.NonIdeal.Kr = kr;
    init.init.NonIdeal.resonantFrequency = resonant_frequency_hz;
    init.init.NonIdeal.cutoffFrequency = cutoff_frequency_hz;
    init.init.NonIdeal.triggerFrequency = trigger_frequency_hz;

    if (VOLOOP_QPR_Init(&qpr_subject->handle, &init) != VOLOOP_OK) {
        return 0;
    }

    vfr_set_qpr_test_subject(qpr_subject, test_subject);

    return 1;
}
