#include "voloop_qpr.h"

static VOLOOP_StatusTypeDef VOLOOP_QPR_ConfigDiscrete(QPR_HandleTypeDef* handle,
                                                      const QPR_InitDiscreteTypeDef* init);
static VOLOOP_StatusTypeDef VOLOOP_QPR_ConfigIdeal(QPR_HandleTypeDef* handle,
                                                   const QPR_InitIdealTypeDef* init);
static VOLOOP_StatusTypeDef VOLOOP_QPR_ConfigNonIdeal(QPR_HandleTypeDef* handle,
                                                      const QPR_InitNonIdealTypeDef* init);

/*
    Tustin (bilinear) discretization of a general second-order continuous
    transfer function:

        G(s) = (B2*s^2 + B1*s + B0) / (A2*s^2 + A1*s + A0)

    Substituting s = c * (z - 1) / (z + 1), with c = 2 / T, and multiplying
    numerator and denominator by (z + 1)^2 yields:

        H(z) = (b0 + b1*z^-1 + b2*z^-2) / (a0 + a1*z^-1 + a2*z^-2)

    where:
        b0 =  B2*c^2 + B1*c + B0
        b1 = -2*B2*c^2        + 2*B0
        b2 =  B2*c^2 - B1*c + B0
        a0 =  A2*c^2 + A1*c + A0
        a1 = -2*A2*c^2        + 2*A0
        a2 =  A2*c^2 - A1*c + A0

    The handle stores coefficients normalized so that a0 = 1.
*/
static VOLOOP_StatusTypeDef VOLOOP_QPR_Tustin(QPR_HandleTypeDef* handle, float B2, float B1,
                                              float B0, float A2, float A1, float A0,
                                              float triggerFrequency) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    if (triggerFrequency <= 0.0f) {
        return VOLOOP_INVALID_PARAM;
    }

    float c = 2.0f * triggerFrequency; // c = 2 / T
    float cc = c * c;

    float b0 = (B2 * cc) + (B1 * c) + B0;
    float b1 = (-2.0f * B2 * cc) + (2.0f * B0);
    float b2 = (B2 * cc) - (B1 * c) + B0;
    float a0 = (A2 * cc) + (A1 * c) + A0;
    float a1 = (-2.0f * A2 * cc) + (2.0f * A0);
    float a2 = (A2 * cc) - (A1 * c) + A0;

    if (a0 == 0.0f) {
        return VOLOOP_INVALID_PARAM;
    }

    float a0Inv = 1.0f / a0;
    handle->b0 = b0 * a0Inv;
    handle->b1 = b1 * a0Inv;
    handle->b2 = b2 * a0Inv;
    handle->a1 = a1 * a0Inv;
    handle->a2 = a2 * a0Inv;

    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_QPR_Init(QPR_HandleTypeDef* handle, const QPR_InitTypeDef* init) {
    if (handle == NULL || init == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    VOLOOP_StatusTypeDef status;
    switch (init->mode) {
        case QPR_Discrete:
            status = VOLOOP_QPR_ConfigDiscrete(handle, &init->init.Discrete);
            break;
        case QPR_Ideal:
            status = VOLOOP_QPR_ConfigIdeal(handle, &init->init.Ideal);
            break;
        case QPR_NonIdeal:
            status = VOLOOP_QPR_ConfigNonIdeal(handle, &init->init.NonIdeal);
            break;
        default:
            return VOLOOP_INVALID_PARAM;
    }
    if (status != VOLOOP_OK) {
        return status;
    }
    return VOLOOP_QPR_Reset(handle);
}

static VOLOOP_StatusTypeDef VOLOOP_QPR_ConfigDiscrete(QPR_HandleTypeDef* handle,
                                                      const QPR_InitDiscreteTypeDef* init) {
    if (handle == NULL || init == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    handle->b0 = init->b0;
    handle->b1 = init->b1;
    handle->b2 = init->b2;
    handle->a1 = init->a1;
    handle->a2 = init->a2;

    return VOLOOP_OK;
}

static VOLOOP_StatusTypeDef VOLOOP_QPR_ConfigIdeal(QPR_HandleTypeDef* handle,
                                                   const QPR_InitIdealTypeDef* init) {
    if (handle == NULL || init == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    if (init->resonantFrequency <= 0.0f || init->triggerFrequency <= 0.0f) {
        return VOLOOP_INVALID_PARAM;
    }

    /*
        Ideal PR:
            G(s) = Kp + Kr * s / (s^2 + w0^2)
                 = (Kp*s^2 + Kr*s + Kp*w0^2) / (s^2 + w0^2)
        So:
            B2 = Kp, B1 = Kr, B0 = Kp*w0^2
            A2 = 1,  A1 = 0,  A0 = w0^2
    */
    float w0 = VOLOOP_TwoPi * init->resonantFrequency;
    float w0Sq = w0 * w0;

    float B2 = init->Kp;
    float B1 = init->Kr;
    float B0 = init->Kp * w0Sq;
    float A2 = 1.0f;
    float A1 = 0.0f;
    float A0 = w0Sq;

    return VOLOOP_QPR_Tustin(handle, B2, B1, B0, A2, A1, A0, init->triggerFrequency);
}

static VOLOOP_StatusTypeDef VOLOOP_QPR_ConfigNonIdeal(QPR_HandleTypeDef* handle,
                                                      const QPR_InitNonIdealTypeDef* init) {
    if (handle == NULL || init == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    if (init->resonantFrequency <= 0.0f || init->cutoffFrequency <= 0.0f ||
        init->triggerFrequency <= 0.0f) {
        return VOLOOP_INVALID_PARAM;
    }

    /*
        Non-ideal / quasi PR:
            G(s) = Kp + Kr * (2*wc*s) / (s^2 + 2*wc*s + w0^2)
                 = (Kp*s^2 + (2*wc*Kp + 2*wc*Kr)*s + Kp*w0^2)
                   / (s^2 + 2*wc*s + w0^2)
        So:
            B2 = Kp, B1 = 2*wc*(Kp + Kr), B0 = Kp*w0^2
            A2 = 1,  A1 = 2*wc,           A0 = w0^2
    */
    float w0 = VOLOOP_TwoPi * init->resonantFrequency;
    float wc = VOLOOP_TwoPi * init->cutoffFrequency;
    float w0Sq = w0 * w0;

    float B2 = init->Kp;
    float B1 = 2.0f * wc * (init->Kp + init->Kr);
    float B0 = init->Kp * w0Sq;
    float A2 = 1.0f;
    float A1 = 2.0f * wc;
    float A0 = w0Sq;

    return VOLOOP_QPR_Tustin(handle, B2, B1, B0, A2, A1, A0, init->triggerFrequency);
}

VOLOOP_StatusTypeDef VOLOOP_QPR_DeInit(QPR_HandleTypeDef* handle) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    *handle = (QPR_HandleTypeDef){ 0 };
    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_QPR_Reconfig(QPR_HandleTypeDef* handle, const QPR_InitTypeDef* init) {
    if (handle == NULL || init == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    switch (init->mode) {
        case QPR_Discrete:
            return VOLOOP_QPR_ConfigDiscrete(handle, &init->init.Discrete);
        case QPR_Ideal:
            return VOLOOP_QPR_ConfigIdeal(handle, &init->init.Ideal);
        case QPR_NonIdeal:
            return VOLOOP_QPR_ConfigNonIdeal(handle, &init->init.NonIdeal);
        default:
            return VOLOOP_INVALID_PARAM;
    }
}

VOLOOP_StatusTypeDef VOLOOP_QPR_Reset(QPR_HandleTypeDef* handle) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    handle->x1 = 0.0f;
    handle->x2 = 0.0f;
    handle->y1 = 0.0f;
    handle->y2 = 0.0f;
    handle->State = QPR_Unsaturated;
    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_QPR_ResetWithValue(QPR_HandleTypeDef* handle, float x1, float x2,
                                               float y1, float y2) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    handle->x1 = x1;
    handle->x2 = x2;
    handle->y1 = y1;
    handle->y2 = y2;
    handle->State = QPR_Unsaturated;

    return VOLOOP_OK;
}

QPR_StateTypeDef VOLOOP_QPR_GetState(QPR_HandleTypeDef* handle) {
    if (handle == NULL) {
        return QPR_ERROR;
    }
    return handle->State;
}

float VOLOOP_QPR_Compute(QPR_HandleTypeDef* handle, float input) {
    if (handle == NULL) {
        return 0.0f; // or some error code
    }

    // Difference equation:
    // H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)
    // y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
    float output = handle->b0 * input + handle->b1 * handle->x1 + handle->b2 * handle->x2 -
                   handle->a1 * handle->y1 - handle->a2 * handle->y2;

    // Update state
    handle->x2 = handle->x1;
    handle->x1 = input;
    handle->y2 = handle->y1;
    handle->y1 = output;
    handle->State = QPR_Unsaturated;

    return output;
}

float VOLOOP_QPR_ComputeBackCalculation(QPR_HandleTypeDef* handle, float input, float outputMin,
                                        float outputMax, float Kb) {
    if (handle == NULL) {
        return 0.0f;
    }

    // Difference equation:
    // H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)
    // y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
    float rawOutput = handle->b0 * input + handle->b1 * handle->x1 + handle->b2 * handle->x2 -
                      handle->a1 * handle->y1 - handle->a2 * handle->y2;
    float output = rawOutput;

    if (rawOutput > outputMax) {
        output = outputMax;
        handle->State = QPR_UpperSaturated;
    } else if (rawOutput < outputMin) {
        output = outputMin;
        handle->State = QPR_LowerSaturated;
    } else {
        handle->State = QPR_Unsaturated;
    }

    float correctedOutput = rawOutput + Kb * (output - rawOutput);

    // Update state with the back-calculated output so the recursive path does not keep winding up.
    handle->x2 = handle->x1;
    handle->x1 = input;
    handle->y2 = handle->y1;
    handle->y1 = correctedOutput;

    return output;
}
