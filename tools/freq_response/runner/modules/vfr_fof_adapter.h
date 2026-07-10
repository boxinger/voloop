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

#endif /* VFR_FOF_ADAPTER_H */
