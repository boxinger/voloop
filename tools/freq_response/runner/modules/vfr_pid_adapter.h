#ifndef VFR_PID_ADAPTER_H
#define VFR_PID_ADAPTER_H

/**
 * @file vfr_pid_adapter.h
 * @brief PID adapters for VOLOOP Frequency Response runner subjects.
 */

#include "vfr_test_subject.h"
#include "voloop_pid.h"

typedef struct {
    PID_HandleTypeDef handle;
} VFR_PidSubject;

int VFR_InitPidDiscreteSubject(VFR_PidSubject* pid_subject,
                               VFR_TestSubject* test_subject,
                               float kp_discrete,
                               float ki_discrete,
                               float kd_discrete);

int VFR_InitPidContinueSubject(VFR_PidSubject* pid_subject,
                               VFR_TestSubject* test_subject,
                               float kp,
                               float ki,
                               float kd,
                               uint32_t trigger_frequency_hz);

int VFR_InitPidOneZeroSubject(VFR_PidSubject* pid_subject,
                              VFR_TestSubject* test_subject,
                              float gain,
                              float zero_hz,
                              uint32_t trigger_frequency_hz);

int VFR_InitPidTwoZeroSubject(VFR_PidSubject* pid_subject,
                              VFR_TestSubject* test_subject,
                              float gain,
                              float zero1_hz,
                              float zero2_hz,
                              uint32_t trigger_frequency_hz);

#endif /* VFR_PID_ADAPTER_H */
