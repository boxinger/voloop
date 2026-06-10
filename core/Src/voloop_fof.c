#include "voloop_fof.h"

static VOLOOP_StatusTypeDef VOLOOP_FOF_ConfigDiscrete(FOF_HandlerTypeDef* handle, const FOF_InitDiscreteTypeDef* init);
static VOLOOP_StatusTypeDef VOLOOP_FOF_ConfigContinue(FOF_HandlerTypeDef* handle, const FOF_InitContinueTypeDef* init);
static VOLOOP_StatusTypeDef VOLOOP_FOF_ConfigLowPass(FOF_HandlerTypeDef* handle, const FOF_InitLowPassTypeDef* init);
static VOLOOP_StatusTypeDef VOLOOP_FOF_ConfigHighPass(FOF_HandlerTypeDef* handle, const FOF_InitHighPassTypeDef* init);
static VOLOOP_StatusTypeDef VOLOOP_FOF_ConfigLeadLag(FOF_HandlerTypeDef* handle, const FOF_InitLeadLagTypeDef* init);

VOLOOP_StatusTypeDef VOLOOP_FOF_Init(FOF_HandlerTypeDef* handle, const FOF_InitTypeDef* init){
    if(handle == NULL || init == NULL){
        return VOLOOP_INVALID_PARAM;
    }

    VOLOOP_StatusTypeDef status;
    switch (init->mode) {
        case FOF_Discrete:
            status = VOLOOP_FOF_ConfigDiscrete(handle, &init->init.Discrete);
            break;
        case FOF_Continue:
            status = VOLOOP_FOF_ConfigContinue(handle, &init->init.Continue);
            break;
        case FOF_LowPass:
            status = VOLOOP_FOF_ConfigLowPass(handle, &init->init.LowPass);
            break;
        case FOF_HighPass:
            status = VOLOOP_FOF_ConfigHighPass(handle, &init->init.HighPass);
            break;
        case FOF_LeadLag:
            status = VOLOOP_FOF_ConfigLeadLag(handle, &init->init.LeadLag);
            break;
        default:
            return VOLOOP_INVALID_PARAM;
    }
    if(status != VOLOOP_OK){
        return status;
    }
    return VOLOOP_FOF_Reset(handle);
}


static VOLOOP_StatusTypeDef VOLOOP_FOF_ConfigDiscrete(FOF_HandlerTypeDef* handle, const FOF_InitDiscreteTypeDef* init){
    if(handle == NULL || init == NULL){
        return VOLOOP_INVALID_PARAM;
    }

    handle->b0 = init->b0;
    handle->b1 = init->b1;
    handle->a1 = init->a1;

    return VOLOOP_OK;
}


static VOLOOP_StatusTypeDef VOLOOP_FOF_ConfigContinue(FOF_HandlerTypeDef* handle, const FOF_InitContinueTypeDef* init){
    if(handle == NULL || init == NULL){
        return VOLOOP_INVALID_PARAM;
    }
    if(init->triggerFrequency <= 0.0f){
        return VOLOOP_INVALID_PARAM;
    }

    /*
        Tustin Approximation
        H(z) = H(s) | s = (2/T) * (z - 1) / (z + 1)
        H(z) = (b0' + b1' * z^-1) / (1 + a1' * z^-1)
        H(s) = K * (b0 * s + b1) / (a0 * s + a1)
                (b1*T + 2*b0) + (b1*T - 2*b0)*z^-1
        H(z) =  --------------------------------------
                (a1*T + 2*a0) + (a1*T - 2*a0)*z^-1
        So:
        b0' = (b1*T + 2*b0) / (a1*T + 2*a0) * K
        b1' = (b1*T - 2*b0) / (a1*T + 2*a0) * K
        a1' = (a1*T - 2*a0) / (a1*T + 2*a0)
    */
    float T = 1.0f / init->triggerFrequency;
    float den = (init->a1 * T) + (2.0f * init->a0);
    if(den == 0.0f){
        return VOLOOP_INVALID_PARAM;
    }
    handle->b0 = ( (init->b1 * T) + (2.0f * init->b0) ) / den * init->K;
    handle->b1 = ( (init->b1 * T) - (2.0f * init->b0) ) / den * init->K;
    handle->a1 = ( (init->a1 * T) - (2.0f * init->a0) ) / den;

    return VOLOOP_OK;
}

static VOLOOP_StatusTypeDef VOLOOP_FOF_ConfigLowPass(FOF_HandlerTypeDef* handle, const FOF_InitLowPassTypeDef* init){
    if(handle == NULL || init == NULL){
        return VOLOOP_INVALID_PARAM;
    }
    if(init->cutoffFrequency <= 0.0f || init->triggerFrequency <= 0.0f){
        return VOLOOP_INVALID_PARAM;
    }

    FOF_InitContinueTypeDef continueInit;
    float OmegaC = VOLOOP_TwoPi * init->cutoffFrequency;
    continueInit.K  = 1.0f;
    continueInit.b0 = 0.0f;
    continueInit.b1 = OmegaC;
    continueInit.a0 = 1.0f;
    continueInit.a1 = OmegaC;
    continueInit.triggerFrequency = init->triggerFrequency;

    return VOLOOP_FOF_ConfigContinue(handle, &continueInit);
}


static VOLOOP_StatusTypeDef VOLOOP_FOF_ConfigHighPass(FOF_HandlerTypeDef* handle, const FOF_InitHighPassTypeDef* init){
    if(handle == NULL || init == NULL){
        return VOLOOP_INVALID_PARAM;
    }
    if(init->cutoffFrequency <= 0.0f || init->triggerFrequency <= 0.0f){
        return VOLOOP_INVALID_PARAM;
    }

    FOF_InitContinueTypeDef continueInit;
    float OmegaC = VOLOOP_TwoPi * init->cutoffFrequency;
    continueInit.K  = 1.0f;
    continueInit.b0 = 1.0f;
    continueInit.b1 = 0.0f;
    continueInit.a0 = 1.0f;
    continueInit.a1 = OmegaC;
    continueInit.triggerFrequency = init->triggerFrequency;

    return VOLOOP_FOF_ConfigContinue(handle, &continueInit);
}


static VOLOOP_StatusTypeDef VOLOOP_FOF_ConfigLeadLag(FOF_HandlerTypeDef* handle, const FOF_InitLeadLagTypeDef* init){
    if(handle == NULL || init == NULL){
        return VOLOOP_INVALID_PARAM;
    }
    if(init->triggerFrequency <= 0.0f
        || init->pole <= 0.0f
        || init->zero <= 0.0f){
        return VOLOOP_INVALID_PARAM;
    }

    FOF_InitContinueTypeDef continueInit;
    float OmegaZ = VOLOOP_TwoPi * init->zero;
    float OmegaP = VOLOOP_TwoPi * init->pole;
    continueInit.K  = init->gain;
    continueInit.b0 = 1.0f;
    continueInit.b1 = OmegaZ;
    continueInit.a0 = 1.0f;
    continueInit.a1 = OmegaP;
    continueInit.triggerFrequency = init->triggerFrequency;

    return VOLOOP_FOF_ConfigContinue(handle, &continueInit);
}

VOLOOP_StatusTypeDef VOLOOP_FOF_DeInit(FOF_HandlerTypeDef* handle){
    if (handle == NULL){
        return VOLOOP_INVALID_PARAM;
    }
    *handle = (FOF_HandlerTypeDef){0};
    return VOLOOP_OK;
}


VOLOOP_StatusTypeDef VOLOOP_FOF_Reconfig(FOF_HandlerTypeDef* handle, const FOF_InitTypeDef* init){
    if(handle == NULL || init == NULL){
        return VOLOOP_INVALID_PARAM;
    }

    switch (init->mode) {
        case FOF_Discrete:
            return VOLOOP_FOF_ConfigDiscrete(handle, &init->init.Discrete);
        case FOF_Continue:
            return VOLOOP_FOF_ConfigContinue(handle, &init->init.Continue);
        case FOF_LowPass:
            return VOLOOP_FOF_ConfigLowPass(handle, &init->init.LowPass);
        case FOF_HighPass:
            return VOLOOP_FOF_ConfigHighPass(handle, &init->init.HighPass);
        case FOF_LeadLag:
            return VOLOOP_FOF_ConfigLeadLag(handle, &init->init.LeadLag);
        default:
            return VOLOOP_INVALID_PARAM;
    }
}


VOLOOP_StatusTypeDef VOLOOP_FOF_Reset(FOF_HandlerTypeDef* handle){
    if(handle == NULL){
        return VOLOOP_INVALID_PARAM;
    }
    handle->x1 = 0.0f;
    handle->y1 = 0.0f;
    return VOLOOP_OK;
}


VOLOOP_StatusTypeDef VOLOOP_FOF_ResetWithValue(
    FOF_HandlerTypeDef* handle,
    float input,
    float output
) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    handle->x1 = input;
    handle->y1 = output;

    return VOLOOP_OK;
}



float VOLOOP_FOF_Compute(FOF_HandlerTypeDef* handle, float input){
    if(handle == NULL){
        return 0.0f; // or some error code
    }
    
    // Difference equation:
    // H(z) = (b0 + b1*z^-1) / (1 + a1*z^-1)
    // y[n] = b0*x[n] + b1*x[n-1] - a1*y[n-1]
    float output = handle->b0 * input + handle->b1 * handle->x1 - handle->a1 * handle->y1;

    // Update state
    handle->x1 = input;
    handle->y1 = output;

    return output;
}
