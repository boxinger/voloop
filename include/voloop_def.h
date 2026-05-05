#ifndef VOLOOP_DEF_H
#define VOLOOP_DEF_H

#include <stdint.h>

#define VOLOOP_Pi 3.14159265358979323846f
#define VOLOOP_TwoPi (2.0f * VOLOOP_Pi)
#define VOLOOP_FourPiSquared (4.0f * VOLOOP_Pi * VOLOOP_Pi)

typedef enum {
    VOLOOP_OK = 0x00U,
    VOLOOP_ERROR,
    VOLOOP_BAD_ALLOCATE,
    VOLOOP_INVALID_PARAM,
    VOLOOP_INVALID_STATE,
    VOLOOP_BUSY,
    VOLOOP_TIMEOUT
} VOLOOP_StatusTypeDef;

/* ===== Unit-tagged scalar types for compile-time dimensional safety ===== */
/* Algorithm layer works with plain float.
   Hardware abstraction layer uses these tagged types to prevent
   dimensional confusion (e.g. passing voltage where current expected). */

typedef struct { float value; } VOLOOP_Voltage;
typedef struct { float value; } VOLOOP_Current;
typedef struct { float value; } VOLOOP_Duty;
typedef struct { float value; } VOLOOP_Frequency;
typedef struct { float value; } VOLOOP_Resistance;
typedef struct { float value; } VOLOOP_Power;

#define VOLOOP_VOLTAGE(v)   ((VOLOOP_Voltage){ .value = (float)(v) })
#define VOLOOP_CURRENT(v)   ((VOLOOP_Current){ .value = (float)(v) })
#define VOLOOP_DUTY(d)      ((VOLOOP_Duty){ .value = (float)(d) })
#define VOLOOP_FREQUENCY(f) ((VOLOOP_Frequency){ .value = (float)(f) })

#define VOLOOP_UNIT_VAL(u)  ((u).value)

#endif /* VOLOOP_DEF_H */
