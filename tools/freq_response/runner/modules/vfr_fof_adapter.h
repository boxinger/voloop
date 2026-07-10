#ifndef VFR_FOF_ADAPTER_H
#define VFR_FOF_ADAPTER_H

/**
 * @file vfr_fof_adapter.h
 * @brief FOF adapters for VOLOOP Frequency Response runner subjects.
 */

#include "vfr_test_subject.h"
#include "voloop_fof.h"

typedef struct {
    FOF_HandleTypeDef handle;
} VFR_FofSubject;

int VFR_InitFofDiscreteSubject(VFR_FofSubject* fof_subject,
                               VFR_TestSubject* test_subject,
                               float b0,
                               float b1,
                               float a1);

int VFR_InitFofLowPassSubject(VFR_FofSubject* fof_subject,
                              VFR_TestSubject* test_subject,
                              float cutoff_hz,
                              float trigger_frequency_hz);

int VFR_InitFofHighPassSubject(VFR_FofSubject* fof_subject,
                               VFR_TestSubject* test_subject,
                               float cutoff_hz,
                               float trigger_frequency_hz);

int VFR_InitFofLeadLagSubject(VFR_FofSubject* fof_subject,
                              VFR_TestSubject* test_subject,
                              float zero_hz,
                              float pole_hz,
                              float gain,
                              float trigger_frequency_hz);

int VFR_InitFofContinueSubject(VFR_FofSubject* fof_subject,
                               VFR_TestSubject* test_subject,
                               float K,
                               float b0,
                               float b1,
                               float a0,
                               float a1,
                               float trigger_frequency_hz);

#endif /* VFR_FOF_ADAPTER_H */
