#ifndef __VOLOOP_DEF_H
#define __VOLOOP_DEF_H

#define VOLOOP_Pi 3.14159265358979323846f
#define VOLOOP_TwoPi (2.0f * VOLOOP_Pi)
#define VOLOOP_FourPiSquared (4.0f * VOLOOP_Pi * VOLOOP_Pi)

#include <math.h>
#ifndef VOLOOP_DEF_SIN
#define VOLOOP_DEF_SIN(x) sinf(x)
#endif

#ifndef VOLOOP_DEF_COS
#define VOLOOP_DEF_COS(x) cosf(x)
#endif


typedef enum {
    VOLOOP_OK = 0x00U,
    VOLOOP_ERROR,
    VOLOOP_BAD_ALLOCATE,
    VOLOOP_INVALID_PARAM,
    VOLOOP_INVALID_STATE,
    VOLOOP_BUSY,
    VOLOOP_TIMEOUT
} VOLOOP_StatusTypeDef;

#endif /* __VOLOOP_DEF_H */
