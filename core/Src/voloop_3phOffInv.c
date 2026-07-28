#include "voloop_3phOffInv.h"

#include <math.h>

#define THREEPHOFFINV_OPEN_LOOP_MODULATION 0.50f
#define THREEPHOFFINV_THIRD_HARMONIC_RATIO (1.0f / 6.0f)
#define THREEPHOFFINV_PHASE_120_Q31 0x55555555U

static int VOLOOP_3phOffInv_IsFiniteFloat(float value) {
    return isfinite(value);
}

static int VOLOOP_3phOffInv_IsConfigValid(const ThreePhOffInv_ConfigTypeDef* config) {
    if (config == NULL) {
        return 0;
    }

    if (!VOLOOP_3phOffInv_IsFiniteFloat(config->LineOverVoltageThreshold) ||
        config->LineOverVoltageThreshold <= 0.0f) {
        return 0;
    }
    if (!VOLOOP_3phOffInv_IsFiniteFloat(config->PhaseOverCurrentThreshold) ||
        config->PhaseOverCurrentThreshold <= 0.0f) {
        return 0;
    }
    if (!VOLOOP_3phOffInv_IsFiniteFloat(config->BusUnderVoltageThreshold) ||
        !VOLOOP_3phOffInv_IsFiniteFloat(config->BusOverVoltageThreshold) ||
        config->BusUnderVoltageThreshold < 0.0f ||
        config->BusOverVoltageThreshold <= config->BusUnderVoltageThreshold) {
        return 0;
    }

    return 1;
}

static void VOLOOP_3phOffInv_DisableOutput(ThreePhOffInv_OutputTypeDef* output) {
    output->PhaseAPwmState = VOLOOP_PWM_DISABLED;
    output->PhaseADuty = 0.0f;
    output->PhaseBPwmState = VOLOOP_PWM_DISABLED;
    output->PhaseBDuty = 0.0f;
    output->PhaseCPwmState = VOLOOP_PWM_DISABLED;
    output->PhaseCDuty = 0.0f;
}

static void VOLOOP_3phOffInv_ResetVoltageControllers(ThreePhOffInv_HandleTypeDef* handle) {
    (void)VOLOOP_PID_Reset(&(handle->VoltageDController));
    (void)VOLOOP_PID_Reset(&(handle->VoltageQController));
}

static void VOLOOP_3phOffInv_DeInitVoltageControllers(ThreePhOffInv_HandleTypeDef* handle) {
    (void)VOLOOP_PID_DeInit(&(handle->VoltageDController));
    (void)VOLOOP_PID_DeInit(&(handle->VoltageQController));
}

static void VOLOOP_3phOffInv_EnterFault(ThreePhOffInv_HandleTypeDef* handle,
                                        ThreePhOffInv_FaultCodeTypeDef faultCode) {
    if (VOLOOP_NCO_GetState(&(handle->NCO)) == NCO_RUNNING) {
        (void)VOLOOP_NCO_Stop(&(handle->NCO));
    }

    if (handle->State != THREEPHOFFINV_ERROR ||
        handle->FaultCode == THREEPHOFFINV_FAULT_NONE) {
        handle->FaultCode = faultCode;
    }
    handle->State = THREEPHOFFINV_ERROR;
}

static void VOLOOP_3phOffInv_ClearHandle(ThreePhOffInv_HandleTypeDef* handle) {
    VOLOOP_3phOffInv_DeInitVoltageControllers(handle);
    (void)VOLOOP_NCO_DeInit(&(handle->NCO));
    *handle = (ThreePhOffInv_HandleTypeDef){ 0 };
}

VOLOOP_StatusTypeDef VOLOOP_3phOffInv_Init(ThreePhOffInv_HandleTypeDef* handle,
                                           const ThreePhOffInv_InitTypeDef* init) {
    if (handle == NULL || init == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    if (!VOLOOP_3phOffInv_IsConfigValid(init->Config) ||
        init->VoltageDControllerInit == NULL ||
        init->VoltageQControllerInit == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    *handle = (ThreePhOffInv_HandleTypeDef){ 0 };
    handle->Config = *(init->Config);
    handle->State = THREEPHOFFINV_RESET;
    handle->FaultCode = THREEPHOFFINV_FAULT_NONE;

    VOLOOP_StatusTypeDef status =
        VOLOOP_PID_Init(&(handle->VoltageDController), init->VoltageDControllerInit);
    if (status != VOLOOP_OK) {
        VOLOOP_3phOffInv_ClearHandle(handle);
        return status;
    }

    status = VOLOOP_PID_Init(&(handle->VoltageQController), init->VoltageQControllerInit);
    if (status != VOLOOP_OK) {
        VOLOOP_3phOffInv_ClearHandle(handle);
        return status;
    }

    status = VOLOOP_NCO_Init(&(handle->NCO), init->NCOInit);
    if (status != VOLOOP_OK) {
        VOLOOP_3phOffInv_ClearHandle(handle);
        return status;
    }

    handle->InitialPhaseRad = init->NCOInit->initialRad;
    handle->OutputPhaseQ31 = VOLOOP_NCO_GetPhaseQ31(&(handle->NCO));
    handle->State = THREEPHOFFINV_DISABLED;
    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_3phOffInv_DeInit(ThreePhOffInv_HandleTypeDef* handle) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    VOLOOP_3phOffInv_ClearHandle(handle);
    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_3phOffInv_Start(ThreePhOffInv_HandleTypeDef* handle) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    if (handle->State == THREEPHOFFINV_RUNNING) {
        return VOLOOP_OK;
    }
    if (handle->State != THREEPHOFFINV_DISABLED) {
        return VOLOOP_INVALID_STATE;
    }

    VOLOOP_3phOffInv_ResetVoltageControllers(handle);

    VOLOOP_StatusTypeDef status = VOLOOP_NCO_SetRad(&(handle->NCO), handle->InitialPhaseRad);
    if (status != VOLOOP_OK) {
        VOLOOP_3phOffInv_EnterFault(handle, THREEPHOFFINV_FAULT_NCO);
        return status;
    }

    status = VOLOOP_NCO_Start(&(handle->NCO));
    if (status != VOLOOP_OK) {
        VOLOOP_3phOffInv_EnterFault(handle, THREEPHOFFINV_FAULT_NCO);
        return status;
    }

    handle->OutputPhaseQ31 = VOLOOP_NCO_GetPhaseQ31(&(handle->NCO));
    handle->State = THREEPHOFFINV_RUNNING;
    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_3phOffInv_Stop(ThreePhOffInv_HandleTypeDef* handle) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    if (handle->State == THREEPHOFFINV_DISABLED) {
        VOLOOP_3phOffInv_ResetVoltageControllers(handle);
        return VOLOOP_OK;
    }
    if (handle->State != THREEPHOFFINV_RUNNING) {
        return VOLOOP_INVALID_STATE;
    }

    VOLOOP_StatusTypeDef status = VOLOOP_NCO_Stop(&(handle->NCO));
    if (status != VOLOOP_OK) {
        VOLOOP_3phOffInv_EnterFault(handle, THREEPHOFFINV_FAULT_NCO);
        return status;
    }

    VOLOOP_3phOffInv_ResetVoltageControllers(handle);
    handle->State = THREEPHOFFINV_DISABLED;
    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_3phOffInv_ClearFaultCode(ThreePhOffInv_HandleTypeDef* handle) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    if (handle->State != THREEPHOFFINV_ERROR && handle->State != THREEPHOFFINV_DISABLED) {
        return VOLOOP_INVALID_STATE;
    }

    NCO_StateTypeDef ncoState = VOLOOP_NCO_GetState(&(handle->NCO));
    VOLOOP_StatusTypeDef status = VOLOOP_OK;
    if (ncoState == NCO_RUNNING) {
        status = VOLOOP_NCO_Stop(&(handle->NCO));
    } else if (ncoState == NCO_ERROR) {
        status = VOLOOP_NCO_ClearFaultCode(&(handle->NCO));
    }
    if (status != VOLOOP_OK) {
        VOLOOP_3phOffInv_EnterFault(handle, THREEPHOFFINV_FAULT_NCO);
        return status;
    }

    VOLOOP_3phOffInv_ResetVoltageControllers(handle);
    handle->FaultCode = THREEPHOFFINV_FAULT_NONE;
    handle->State = THREEPHOFFINV_DISABLED;
    return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_3phOffInv_SetFrequency(ThreePhOffInv_HandleTypeDef* handle,
                                                   float frequency) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    if (handle->State != THREEPHOFFINV_DISABLED && handle->State != THREEPHOFFINV_RUNNING) {
        return VOLOOP_INVALID_STATE;
    }
    if (!VOLOOP_3phOffInv_IsFiniteFloat(frequency) || frequency <= 0.0f ||
        frequency >= (float)handle->NCO.Init.triggerFrequency) {
        return VOLOOP_INVALID_PARAM;
    }
    if (VOLOOP_NCO_GetState(&(handle->NCO)) == NCO_ERROR) {
        VOLOOP_3phOffInv_EnterFault(handle, THREEPHOFFINV_FAULT_NCO);
        return VOLOOP_INVALID_STATE;
    }

    return VOLOOP_NCO_SetFrequency(&(handle->NCO), frequency);
}

ThreePhOffInv_StateTypeDef VOLOOP_3phOffInv_GetState(const ThreePhOffInv_HandleTypeDef* handle) {
    if (handle == NULL) {
        return THREEPHOFFINV_ERROR;
    }

    return handle->State;
}

ThreePhOffInv_FaultCodeTypeDef
VOLOOP_3phOffInv_GetFaultCode(const ThreePhOffInv_HandleTypeDef* handle) {
    if (handle == NULL) {
        return THREEPHOFFINV_FAULT_INVALID;
    }

    return handle->FaultCode;
}

float VOLOOP_3phOffInv_GetFrequency(const ThreePhOffInv_HandleTypeDef* handle) {
    if (handle == NULL) {
        return 0.0f;
    }

    return handle->NCO.Frequency;
}

int32_t VOLOOP_3phOffInv_GetPhaseQ31(const ThreePhOffInv_HandleTypeDef* handle) {
    if (handle == NULL) {
        return 0;
    }

    return handle->OutputPhaseQ31;
}

VOLOOP_StatusTypeDef VOLOOP_3phOffInv_Sync(ThreePhOffInv_HandleTypeDef* handle,
                                           const ThreePhOffInv_InputTypeDef* input,
                                           ThreePhOffInv_OutputTypeDef* output) {
    if (output != NULL) {
        VOLOOP_3phOffInv_DisableOutput(output);
    }
    if (handle == NULL || input == NULL || output == NULL) {
        return VOLOOP_INVALID_PARAM;
    }

    if (handle->State == THREEPHOFFINV_DISABLED) {
        return VOLOOP_OK;
    }
    if (handle->State != THREEPHOFFINV_RUNNING) {
        return VOLOOP_INVALID_STATE;
    }

    float lineVoltageBC = input->LineVoltageAC - input->LineVoltageAB;
    float phaseCurrentC = -(input->PhaseCurrentA + input->PhaseCurrentB);
    if (!VOLOOP_3phOffInv_IsFiniteFloat(input->BusVoltage) ||
        !VOLOOP_3phOffInv_IsFiniteFloat(input->LineVoltageAB) ||
        !VOLOOP_3phOffInv_IsFiniteFloat(input->LineVoltageAC) ||
        !VOLOOP_3phOffInv_IsFiniteFloat(input->PhaseCurrentA) ||
        !VOLOOP_3phOffInv_IsFiniteFloat(input->PhaseCurrentB) ||
        !VOLOOP_3phOffInv_IsFiniteFloat(lineVoltageBC) ||
        !VOLOOP_3phOffInv_IsFiniteFloat(phaseCurrentC)) {
        VOLOOP_3phOffInv_EnterFault(handle, THREEPHOFFINV_FAULT_INVALID_SAMPLE);
        return VOLOOP_ERROR;
    }

    if (fabsf(input->LineVoltageAB) > handle->Config.LineOverVoltageThreshold ||
        fabsf(input->LineVoltageAC) > handle->Config.LineOverVoltageThreshold ||
        fabsf(lineVoltageBC) > handle->Config.LineOverVoltageThreshold) {
        VOLOOP_3phOffInv_EnterFault(handle, THREEPHOFFINV_FAULT_LINE_OVERVOLTAGE);
        return VOLOOP_ERROR;
    }
    if (fabsf(input->PhaseCurrentA) > handle->Config.PhaseOverCurrentThreshold ||
        fabsf(input->PhaseCurrentB) > handle->Config.PhaseOverCurrentThreshold ||
        fabsf(phaseCurrentC) > handle->Config.PhaseOverCurrentThreshold) {
        VOLOOP_3phOffInv_EnterFault(handle, THREEPHOFFINV_FAULT_PHASE_OVERCURRENT);
        return VOLOOP_ERROR;
    }
    if (input->BusVoltage < handle->Config.BusUnderVoltageThreshold) {
        VOLOOP_3phOffInv_EnterFault(handle, THREEPHOFFINV_FAULT_BUS_UNDERVOLTAGE);
        return VOLOOP_ERROR;
    }
    if (input->BusVoltage > handle->Config.BusOverVoltageThreshold) {
        VOLOOP_3phOffInv_EnterFault(handle, THREEPHOFFINV_FAULT_BUS_OVERVOLTAGE);
        return VOLOOP_ERROR;
    }

    int32_t outputPhaseQ31 = VOLOOP_NCO_GetPhaseQ31(&(handle->NCO));
    VOLOOP_StatusTypeDef status = VOLOOP_NCO_Sync(&(handle->NCO));
    if (status != VOLOOP_OK) {
        VOLOOP_3phOffInv_EnterFault(handle, THREEPHOFFINV_FAULT_NCO);
        return VOLOOP_ERROR;
    }

    int32_t phaseBQ31 = (int32_t)((uint32_t)outputPhaseQ31 - THREEPHOFFINV_PHASE_120_Q31);
    int32_t phaseCQ31 = (int32_t)((uint32_t)outputPhaseQ31 + THREEPHOFFINV_PHASE_120_Q31);
    int32_t thirdHarmonicPhaseQ31 = (int32_t)((uint32_t)outputPhaseQ31 * 3U);
    float thirdHarmonic =
        THREEPHOFFINV_THIRD_HARMONIC_RATIO * VOLOOP_DEF_SIN(thirdHarmonicPhaseQ31);
    float phaseAReference = VOLOOP_DEF_SIN(outputPhaseQ31) + thirdHarmonic;
    float phaseBReference = VOLOOP_DEF_SIN(phaseBQ31) + thirdHarmonic;
    float phaseCReference = VOLOOP_DEF_SIN(phaseCQ31) + thirdHarmonic;
    float dutyScale = 0.5f * THREEPHOFFINV_OPEN_LOOP_MODULATION;

    output->PhaseAPwmState = VOLOOP_PWM_ENABLE;
    output->PhaseADuty = VOLOOP_DEF_ClampFloat(0.5f + (dutyScale * phaseAReference), 0.0f, 1.0f);
    output->PhaseBPwmState = VOLOOP_PWM_ENABLE;
    output->PhaseBDuty = VOLOOP_DEF_ClampFloat(0.5f + (dutyScale * phaseBReference), 0.0f, 1.0f);
    output->PhaseCPwmState = VOLOOP_PWM_ENABLE;
    output->PhaseCDuty = VOLOOP_DEF_ClampFloat(0.5f + (dutyScale * phaseCReference), 0.0f, 1.0f);

    handle->OutputPhaseQ31 = outputPhaseQ31;
    return VOLOOP_OK;
}
