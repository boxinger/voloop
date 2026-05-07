#ifndef VOLOOP_STM32_ENCODER_H
#define VOLOOP_STM32_ENCODER_H

#include <stdint.h>
#include "voloop_def.h"
#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef VOLOOP_STM32_ENCODER_COUNT_SCALE
#define VOLOOP_STM32_ENCODER_COUNT_SCALE 2  U
#endif

VOLOOP_StatusTypeDef VOLOOP_STM32_Encoder_Init(TIM_HandleTypeDef* htim);
VOLOOP_StatusTypeDef VOLOOP_STM32_Encoder_DeInit(void);

VOLOOP_StatusTypeDef VOLOOP_STM32_Encoder_GetCount(int16_t* outCount);
VOLOOP_StatusTypeDef VOLOOP_STM32_Encoder_SetCount(int16_t count);
VOLOOP_StatusTypeDef VOLOOP_STM32_Encoder_Clear(void);

VOLOOP_StatusTypeDef VOLOOP_STM32_Encoder_PopCount(int16_t* outDelta);

#ifdef __cplusplus
}
#endif

#endif /* VOLOOP_STM32_ENCODER_H */
