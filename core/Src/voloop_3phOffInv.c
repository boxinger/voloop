#include "voloop_3phOffInv.h"

#include <math.h>

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

static void VOLOOP_3phOffInv_ResetControllers(ThreePhOffInv_HandleTypeDef* handle) {
    (void)VOLOOP_PID_Reset(&(handle->VoltageDController));
    (void)VOLOOP_PID_Reset(&(handle->VoltageQController));
    (void)VOLOOP_PID_Reset(&(handle->CurrentDController));
    (void)VOLOOP_PID_Reset(&(handle->CurrentQController));
}

static void VOLOOP_3phOffInv_DeInitControllers(ThreePhOffInv_HandleTypeDef* handle) {
    (void)VOLOOP_PID_DeInit(&(handle->VoltageDController));
    (void)VOLOOP_PID_DeInit(&(handle->VoltageQController));
    (void)VOLOOP_PID_DeInit(&(handle->CurrentDController));
    (void)VOLOOP_PID_DeInit(&(handle->CurrentQController));
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
    VOLOOP_3phOffInv_DeInitControllers(handle);
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
        init->VoltageQControllerInit == NULL ||
        init->CurrentDControllerInit == NULL ||
        init->CurrentQControllerInit == NULL) {
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

    status = VOLOOP_PID_Init(&(handle->CurrentDController), init->CurrentDControllerInit);
    if (status != VOLOOP_OK) {
        VOLOOP_3phOffInv_ClearHandle(handle);
        return status;
    }

    status = VOLOOP_PID_Init(&(handle->CurrentQController), init->CurrentQControllerInit);
    if (status != VOLOOP_OK) {
        VOLOOP_3phOffInv_ClearHandle(handle);
        return status;
    }

    status = VOLOOP_NCO_Init(&(handle->NCO), init->NCOInit);
    if (status != VOLOOP_OK) {
        VOLOOP_3phOffInv_ClearHandle(handle);
        return status;
    }

    handle->TargetLineVoltagePeak = 0.0f;
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

    VOLOOP_3phOffInv_ResetControllers(handle);

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
        VOLOOP_3phOffInv_ResetControllers(handle);
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

    VOLOOP_3phOffInv_ResetControllers(handle);
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

    VOLOOP_3phOffInv_ResetControllers(handle);
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

VOLOOP_StatusTypeDef VOLOOP_3phOffInv_SetLineVoltagePeak(
    ThreePhOffInv_HandleTypeDef* handle,
    float lineVoltagePeak) {
    if (handle == NULL) {
        return VOLOOP_INVALID_PARAM;
    }
    if (handle->State != THREEPHOFFINV_DISABLED &&
        handle->State != THREEPHOFFINV_RUNNING) {
        return VOLOOP_INVALID_STATE;
    }
    if (!VOLOOP_3phOffInv_IsFiniteFloat(lineVoltagePeak) ||
        lineVoltagePeak < 0.0f ||
        lineVoltagePeak > handle->Config.LineOverVoltageThreshold) {
        return VOLOOP_INVALID_PARAM;
    }

    handle->TargetLineVoltagePeak = lineVoltagePeak;
    return VOLOOP_OK;
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

    float lineVoltageAC = input->LineVoltageAB + input->LineVoltageBC;
    float phaseCurrentB = -(input->PhaseCurrentA + input->PhaseCurrentC);
    if (!VOLOOP_3phOffInv_IsFiniteFloat(input->BusVoltage) ||
        !VOLOOP_3phOffInv_IsFiniteFloat(input->LineVoltageAB) ||
        !VOLOOP_3phOffInv_IsFiniteFloat(input->LineVoltageBC) ||
        !VOLOOP_3phOffInv_IsFiniteFloat(input->PhaseCurrentA) ||
        !VOLOOP_3phOffInv_IsFiniteFloat(input->PhaseCurrentC) ||
        !VOLOOP_3phOffInv_IsFiniteFloat(lineVoltageAC) ||
        !VOLOOP_3phOffInv_IsFiniteFloat(phaseCurrentB)) {
        VOLOOP_3phOffInv_EnterFault(handle, THREEPHOFFINV_FAULT_INVALID_SAMPLE);
        return VOLOOP_ERROR;
    }

    if (fabsf(input->LineVoltageAB) > handle->Config.LineOverVoltageThreshold ||
        fabsf(input->LineVoltageBC) > handle->Config.LineOverVoltageThreshold ||
        fabsf(lineVoltageAC) > handle->Config.LineOverVoltageThreshold) {
        VOLOOP_3phOffInv_EnterFault(handle, THREEPHOFFINV_FAULT_LINE_OVERVOLTAGE);
        return VOLOOP_ERROR;
    }
    if (fabsf(input->PhaseCurrentA) > handle->Config.PhaseOverCurrentThreshold ||
        fabsf(input->PhaseCurrentC) > handle->Config.PhaseOverCurrentThreshold ||
        fabsf(phaseCurrentB) > handle->Config.PhaseOverCurrentThreshold) {
        VOLOOP_3phOffInv_EnterFault(handle, THREEPHOFFINV_FAULT_PHASE_OVERCURRENT);
        return VOLOOP_ERROR;
    }
    if (input->BusVoltage <= 0.0f ||
        input->BusVoltage < handle->Config.BusUnderVoltageThreshold) {
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

    const float oneOverSqrt3 = 0.57735026918962576451f;
    const float modulationMax = 1.15470053837925152902f;
    const int32_t quarterCycleQ31 = (int32_t)0x40000000U;

    VOLOOP_DEF_AbcTypeDef measuredPhaseVoltage = {
        .a = (input->LineVoltageAB + lineVoltageAC) * (1.0f / 3.0f),
    };
    measuredPhaseVoltage.b = measuredPhaseVoltage.a - input->LineVoltageAB;
    measuredPhaseVoltage.c = measuredPhaseVoltage.b - input->LineVoltageBC;

    VOLOOP_DEF_AlphaBetaZeroTypeDef measuredAlphaBetaVoltage = { 0 };
    VOLOOP_DEF_ClarkeTransform(&measuredPhaseVoltage, &measuredAlphaBetaVoltage);

    int32_t parkPhaseQ31 =
        (int32_t)((uint32_t)outputPhaseQ31 - (uint32_t)quarterCycleQ31);
    VOLOOP_DEF_DqZeroTypeDef measuredDqVoltage = { 0 };
    VOLOOP_DEF_ParkTransform(&measuredAlphaBetaVoltage, parkPhaseQ31,
                             &measuredDqVoltage);

    VOLOOP_DEF_AbcTypeDef measuredPhaseCurrent = {
        .a = input->PhaseCurrentA,
        .b = phaseCurrentB,
        .c = input->PhaseCurrentC,
    };
    VOLOOP_DEF_AlphaBetaZeroTypeDef measuredAlphaBetaCurrent = { 0 };
    VOLOOP_DEF_ClarkeTransform(&measuredPhaseCurrent, &measuredAlphaBetaCurrent);

    VOLOOP_DEF_DqZeroTypeDef measuredDqCurrent = { 0 };
    VOLOOP_DEF_ParkTransform(&measuredAlphaBetaCurrent, parkPhaseQ31,
                             &measuredDqCurrent);

    float dVoltageReference = handle->TargetLineVoltagePeak * oneOverSqrt3;
    float currentReferenceMax = handle->Config.PhaseOverCurrentThreshold;
    float dCurrentReference = VOLOOP_PID_ComputeConditional(
        &(handle->VoltageDController), dVoltageReference, measuredDqVoltage.d,
        -currentReferenceMax, currentReferenceMax);

    float qCurrentHeadroomSquared =
        (currentReferenceMax * currentReferenceMax) -
        (dCurrentReference * dCurrentReference);
    if (qCurrentHeadroomSquared < 0.0f) {
        qCurrentHeadroomSquared = 0.0f;
    }
    float qCurrentHeadroom = sqrtf(qCurrentHeadroomSquared);
    float qCurrentReference = VOLOOP_PID_ComputeConditional(
        &(handle->VoltageQController), 0.0f, measuredDqVoltage.q,
        -qCurrentHeadroom, qCurrentHeadroom);

    float dModulationFeedforward =
        (2.0f * measuredDqVoltage.d) / input->BusVoltage;
    float qModulationFeedforward =
        (2.0f * measuredDqVoltage.q) / input->BusVoltage;
    float dCurrentCorrection = VOLOOP_PID_ComputeConditional(
        &(handle->CurrentDController), dCurrentReference, measuredDqCurrent.d,
        -modulationMax - dModulationFeedforward,
        modulationMax - dModulationFeedforward);
    float dModulation = VOLOOP_DEF_ClampFloat(
        dModulationFeedforward + dCurrentCorrection,
        -modulationMax, modulationMax);

    float qModulationHeadroomSquared =
        (modulationMax * modulationMax) - (dModulation * dModulation);
    if (qModulationHeadroomSquared < 0.0f) {
        qModulationHeadroomSquared = 0.0f;
    }
    float qModulationHeadroom = sqrtf(qModulationHeadroomSquared);
    float qCurrentCorrection = VOLOOP_PID_ComputeConditional(
        &(handle->CurrentQController), qCurrentReference, measuredDqCurrent.q,
        -qModulationHeadroom - qModulationFeedforward,
        qModulationHeadroom - qModulationFeedforward);
    float qModulation = VOLOOP_DEF_ClampFloat(
        qModulationFeedforward + qCurrentCorrection,
        -qModulationHeadroom, qModulationHeadroom);

    VOLOOP_DEF_DqZeroTypeDef modulationDq = {
        .d = dModulation,
        .q = qModulation,
        .zero = 0.0f,
    };
    VOLOOP_DEF_AlphaBetaZeroTypeDef modulationAlphaBeta = { 0 };
    VOLOOP_DEF_InverseParkTransform(&modulationDq, parkPhaseQ31,
                                    &modulationAlphaBeta);

    VOLOOP_DEF_AbcTypeDef phaseModulation = { 0 };
    VOLOOP_DEF_InverseClarkeTransform(&modulationAlphaBeta, &phaseModulation);

    float modulationMaximum = phaseModulation.a;
    float modulationMinimum = phaseModulation.a;
    if (phaseModulation.b > modulationMaximum) {
        modulationMaximum = phaseModulation.b;
    }
    if (phaseModulation.c > modulationMaximum) {
        modulationMaximum = phaseModulation.c;
    }
    if (phaseModulation.b < modulationMinimum) {
        modulationMinimum = phaseModulation.b;
    }
    if (phaseModulation.c < modulationMinimum) {
        modulationMinimum = phaseModulation.c;
    }
    float commonModeModulation = -0.5f * (modulationMaximum + modulationMinimum);

    output->PhaseAPwmState = VOLOOP_PWM_ENABLE;
    output->PhaseADuty = VOLOOP_DEF_ClampFloat(
        0.5f + (0.5f * (phaseModulation.a + commonModeModulation)),
        0.0f, 1.0f);
    output->PhaseBPwmState = VOLOOP_PWM_ENABLE;
    output->PhaseBDuty = VOLOOP_DEF_ClampFloat(
        0.5f + (0.5f * (phaseModulation.b + commonModeModulation)),
        0.0f, 1.0f);
    output->PhaseCPwmState = VOLOOP_PWM_ENABLE;
    output->PhaseCDuty = VOLOOP_DEF_ClampFloat(
        0.5f + (0.5f * (phaseModulation.c + commonModeModulation)),
        0.0f, 1.0f);

    handle->OutputPhaseQ31 = outputPhaseQ31;
    return VOLOOP_OK;
}
