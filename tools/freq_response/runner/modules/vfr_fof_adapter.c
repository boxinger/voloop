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

    test_subject->context = fof_subject;
    test_subject->compute = vfr_fof_compute;
    test_subject->reset = vfr_fof_reset;

    return 1;
}
