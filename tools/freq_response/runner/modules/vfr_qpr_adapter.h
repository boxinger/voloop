#ifndef VFR_QPR_ADAPTER_H
#define VFR_QPR_ADAPTER_H

/**
 * @file vfr_qpr_adapter.h
 * @brief QPR adapters for VOLOOP Frequency Response runner subjects.
 */

#include "vfr_test_subject.h"
#include "voloop_qpr.h"

typedef struct {
    QPR_HandleTypeDef handle;
} VFR_QprSubject;

int VFR_InitQprDiscreteSubject(VFR_QprSubject* qpr_subject,
                               VFR_TestSubject* test_subject,
                               float b0,
                               float b1,
                               float b2,
                               float a1,
                               float a2);

int VFR_InitQprIdealSubject(VFR_QprSubject* qpr_subject,
                            VFR_TestSubject* test_subject,
                            float kp,
                            float kr,
                            float resonant_frequency_hz,
                            float trigger_frequency_hz);

int VFR_InitQprNonIdealSubject(VFR_QprSubject* qpr_subject,
                               VFR_TestSubject* test_subject,
                               float kp,
                               float kr,
                               float resonant_frequency_hz,
                               float cutoff_frequency_hz,
                               float trigger_frequency_hz);

#endif /* VFR_QPR_ADAPTER_H */
