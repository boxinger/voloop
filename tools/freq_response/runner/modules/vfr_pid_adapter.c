#include "vfr_pid_adapter.h"

#include <stddef.h>

static float vfr_pid_compute(void* context, float input) {
    VFR_PidSubject* subject = (VFR_PidSubject*)context;

    if (subject == NULL) {
        return 0.0f;
    }

    return VOLOOP_PID_Compute(&subject->handle, input, 0.0f);
}

static void vfr_pid_reset(void* context) {
    VFR_PidSubject* subject = (VFR_PidSubject*)context;

    if (subject == NULL) {
        return;
    }

    (void)VOLOOP_PID_Reset(&subject->handle);
}

static void vfr_set_pid_test_subject(VFR_PidSubject* pid_subject,
                                     VFR_TestSubject* test_subject) {
    test_subject->context = pid_subject;
    test_subject->compute = vfr_pid_compute;
    test_subject->reset = vfr_pid_reset;
}

int VFR_InitPidDiscreteSubject(VFR_PidSubject* pid_subject,
                               VFR_TestSubject* test_subject,
                               float kp_discrete,
                               float ki_discrete,
                               float kd_discrete) {
    PID_InitTypeDef init = {0};

    if (pid_subject == NULL || test_subject == NULL) {
        return 0;
    }

    init.mode = PID_Discrete;
    init.init.Discrete.KpDiscrete = kp_discrete;
    init.init.Discrete.KiDiscrete = ki_discrete;
    init.init.Discrete.KdDiscrete = kd_discrete;

    if (VOLOOP_PID_Init(&pid_subject->handle, &init) != VOLOOP_OK) {
        return 0;
    }

    vfr_set_pid_test_subject(pid_subject, test_subject);

    return 1;
}

int VFR_InitPidContinueSubject(VFR_PidSubject* pid_subject,
                               VFR_TestSubject* test_subject,
                               float kp,
                               float ki,
                               float kd,
                               uint32_t trigger_frequency_hz) {
    PID_InitTypeDef init = {0};

    if (pid_subject == NULL || test_subject == NULL) {
        return 0;
    }

    init.mode = PID_Continue;
    init.init.Continue.Kp = kp;
    init.init.Continue.Ki = ki;
    init.init.Continue.Kd = kd;
    init.init.Continue.triggerFrequency = trigger_frequency_hz;

    if (VOLOOP_PID_Init(&pid_subject->handle, &init) != VOLOOP_OK) {
        return 0;
    }

    vfr_set_pid_test_subject(pid_subject, test_subject);

    return 1;
}

int VFR_InitPidOneZeroSubject(VFR_PidSubject* pid_subject,
                              VFR_TestSubject* test_subject,
                              float gain,
                              float zero_hz,
                              uint32_t trigger_frequency_hz) {
    PID_InitTypeDef init = {0};

    if (pid_subject == NULL || test_subject == NULL) {
        return 0;
    }

    init.mode = PID_OneZero;
    init.init.OneZero.gain = gain;
    init.init.OneZero.zero = zero_hz;
    init.init.OneZero.triggerFrequency = trigger_frequency_hz;

    if (VOLOOP_PID_Init(&pid_subject->handle, &init) != VOLOOP_OK) {
        return 0;
    }

    vfr_set_pid_test_subject(pid_subject, test_subject);

    return 1;
}

int VFR_InitPidTwoZeroSubject(VFR_PidSubject* pid_subject,
                              VFR_TestSubject* test_subject,
                              float gain,
                              float zero1_hz,
                              float zero2_hz,
                              uint32_t trigger_frequency_hz) {
    PID_InitTypeDef init = {0};

    if (pid_subject == NULL || test_subject == NULL) {
        return 0;
    }

    init.mode = PID_TwoZero;
    init.init.TwoZero.gain = gain;
    init.init.TwoZero.zero1 = zero1_hz;
    init.init.TwoZero.zero2 = zero2_hz;
    init.init.TwoZero.triggerFrequency = trigger_frequency_hz;

    if (VOLOOP_PID_Init(&pid_subject->handle, &init) != VOLOOP_OK) {
        return 0;
    }

    vfr_set_pid_test_subject(pid_subject, test_subject);

    return 1;
}
