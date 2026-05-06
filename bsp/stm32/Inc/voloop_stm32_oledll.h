#ifndef VOLOOP_STM32_OLEDLL_H
#define VOLOOP_STM32_OLEDLL_H

#include <stdint.h>
#include "voloop_def.h"
#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define VOLOOP_STM32_OLEDLL_WIDTH 128U
#define VOLOOP_STM32_OLEDLL_HEIGHT 64U
#define VOLOOP_STM32_OLEDLL_PAGE_COUNT (VOLOOP_STM32_OLEDLL_HEIGHT / 8U)
#define VOLOOP_STM32_OLEDLL_FRAME_BYTES (VOLOOP_STM32_OLEDLL_WIDTH * VOLOOP_STM32_OLEDLL_PAGE_COUNT)

#define VOLOOP_STM32_OLEDLL_DEFAULT_ADDR 0x78U
#define VOLOOP_STM32_OLEDLL_DEFAULT_TIMEOUT_CMD 10U
#define VOLOOP_STM32_OLEDLL_DEFAULT_TIMEOUT_DATA 100U

typedef struct {
	I2C_HandleTypeDef* hi2c;
	uint16_t devAddr;
	uint32_t timeoutCmd;
	uint32_t timeoutData;
} VOLOOP_STM32_OLEDLL_InitTypeDef;

VOLOOP_StatusTypeDef VOLOOP_STM32_OLEDLL_Init(const VOLOOP_STM32_OLEDLL_InitTypeDef* init);
VOLOOP_StatusTypeDef VOLOOP_STM32_OLEDLL_Start(void);
VOLOOP_StatusTypeDef VOLOOP_STM32_OLEDLL_Stop(void);
VOLOOP_StatusTypeDef VOLOOP_STM32_OLEDLL_Clear(void);
VOLOOP_StatusTypeDef VOLOOP_STM32_OLEDLL_Refresh(const uint8_t* buffer, uint16_t length);

#ifdef __cplusplus
}
#endif


#endif /* VOLOOP_STM32_OLEDLL_H */
