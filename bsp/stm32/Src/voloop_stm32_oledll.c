#include "voloop_stm32_oledll.h"

#define VOLOOP_STM32_OLEDLL_CTRL_CMD 0x00U
#define VOLOOP_STM32_OLEDLL_CTRL_DATA 0x40U
#define VOLOOP_STM32_OLEDLL_CLEAR_CHUNK 16U

typedef struct {
	I2C_HandleTypeDef* hi2c;
	uint16_t devAddr;
	uint32_t timeoutCmd;
	uint32_t timeoutData;
	uint8_t isInitialized;
} VOLOOP_STM32_OLEDLL_ContextTypeDef;

static VOLOOP_STM32_OLEDLL_ContextTypeDef s_oledCtx = {0};

static VOLOOP_StatusTypeDef VOLOOP_STM32_OLEDLL_WriteCmd(uint8_t cmd) {
	if (s_oledCtx.hi2c == NULL) {
		return VOLOOP_INVALID_STATE;
	}

	if (HAL_I2C_Mem_Write(s_oledCtx.hi2c,
						  s_oledCtx.devAddr,
						  VOLOOP_STM32_OLEDLL_CTRL_CMD,
						  I2C_MEMADD_SIZE_8BIT,
						  &cmd,
						  1U,
						  s_oledCtx.timeoutCmd) != HAL_OK) {
		return VOLOOP_ERROR;
	}

	return VOLOOP_OK;
}

static VOLOOP_StatusTypeDef VOLOOP_STM32_OLEDLL_WriteDataStream(const uint8_t* data, uint16_t len) {
	if (s_oledCtx.hi2c == NULL || data == NULL || len == 0U) {
		return VOLOOP_INVALID_PARAM;
	}

	if (HAL_I2C_Mem_Write(s_oledCtx.hi2c,
						  s_oledCtx.devAddr,
						  VOLOOP_STM32_OLEDLL_CTRL_DATA,
						  I2C_MEMADD_SIZE_8BIT,
						  (uint8_t*)data,
						  len,
						  s_oledCtx.timeoutData) != HAL_OK) {
		return VOLOOP_ERROR;
	}

	return VOLOOP_OK;
}

static VOLOOP_StatusTypeDef VOLOOP_STM32_OLEDLL_SetAddrWindow(uint8_t colStart,
															   uint8_t colEnd,
															   uint8_t pageStart,
															   uint8_t pageEnd) {
	VOLOOP_StatusTypeDef status;

	status = VOLOOP_STM32_OLEDLL_WriteCmd(0x21U);
	if (status != VOLOOP_OK) return status;
	status = VOLOOP_STM32_OLEDLL_WriteCmd(colStart);
	if (status != VOLOOP_OK) return status;
	status = VOLOOP_STM32_OLEDLL_WriteCmd(colEnd);
	if (status != VOLOOP_OK) return status;

	status = VOLOOP_STM32_OLEDLL_WriteCmd(0x22U);
	if (status != VOLOOP_OK) return status;
	status = VOLOOP_STM32_OLEDLL_WriteCmd(pageStart);
	if (status != VOLOOP_OK) return status;
	status = VOLOOP_STM32_OLEDLL_WriteCmd(pageEnd);
	if (status != VOLOOP_OK) return status;

	return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_STM32_OLEDLL_Init(const VOLOOP_STM32_OLEDLL_InitTypeDef* init) {
	VOLOOP_StatusTypeDef status;

	if (init == NULL || init->hi2c == NULL) {
		return VOLOOP_INVALID_PARAM;
	}

	s_oledCtx.hi2c = init->hi2c;
	s_oledCtx.devAddr = (init->devAddr == 0U) ? VOLOOP_STM32_OLEDLL_DEFAULT_ADDR : init->devAddr;
	s_oledCtx.timeoutCmd = (init->timeoutCmd == 0U) ? VOLOOP_STM32_OLEDLL_DEFAULT_TIMEOUT_CMD : init->timeoutCmd;
	s_oledCtx.timeoutData = (init->timeoutData == 0U) ? VOLOOP_STM32_OLEDLL_DEFAULT_TIMEOUT_DATA : init->timeoutData;
	s_oledCtx.isInitialized = 1U;

	HAL_Delay(100U);

	status = VOLOOP_STM32_OLEDLL_WriteCmd(0xAEU);
	if (status != VOLOOP_OK) return status;

	status = VOLOOP_STM32_OLEDLL_WriteCmd(0x20U);
	if (status != VOLOOP_OK) return status;
	status = VOLOOP_STM32_OLEDLL_WriteCmd(0x00U);
	if (status != VOLOOP_OK) return status;

	status = VOLOOP_STM32_OLEDLL_WriteCmd(0xB0U);
	if (status != VOLOOP_OK) return status;
	status = VOLOOP_STM32_OLEDLL_WriteCmd(0xC8U);
	if (status != VOLOOP_OK) return status;

	status = VOLOOP_STM32_OLEDLL_WriteCmd(0x00U);
	if (status != VOLOOP_OK) return status;
	status = VOLOOP_STM32_OLEDLL_WriteCmd(0x10U);
	if (status != VOLOOP_OK) return status;

	status = VOLOOP_STM32_OLEDLL_WriteCmd(0x40U);
	if (status != VOLOOP_OK) return status;

	status = VOLOOP_STM32_OLEDLL_WriteCmd(0x81U);
	if (status != VOLOOP_OK) return status;
	status = VOLOOP_STM32_OLEDLL_WriteCmd(0xCFU);
	if (status != VOLOOP_OK) return status;

	status = VOLOOP_STM32_OLEDLL_WriteCmd(0xA1U);
	if (status != VOLOOP_OK) return status;

	status = VOLOOP_STM32_OLEDLL_WriteCmd(0xA6U);
	if (status != VOLOOP_OK) return status;

	status = VOLOOP_STM32_OLEDLL_WriteCmd(0xA8U);
	if (status != VOLOOP_OK) return status;
	status = VOLOOP_STM32_OLEDLL_WriteCmd(0x3FU);
	if (status != VOLOOP_OK) return status;

	status = VOLOOP_STM32_OLEDLL_WriteCmd(0xA4U);
	if (status != VOLOOP_OK) return status;

	status = VOLOOP_STM32_OLEDLL_WriteCmd(0xD3U);
	if (status != VOLOOP_OK) return status;
	status = VOLOOP_STM32_OLEDLL_WriteCmd(0x00U);
	if (status != VOLOOP_OK) return status;

	status = VOLOOP_STM32_OLEDLL_WriteCmd(0xD5U);
	if (status != VOLOOP_OK) return status;
	status = VOLOOP_STM32_OLEDLL_WriteCmd(0x80U);
	if (status != VOLOOP_OK) return status;

	status = VOLOOP_STM32_OLEDLL_WriteCmd(0xD9U);
	if (status != VOLOOP_OK) return status;
	status = VOLOOP_STM32_OLEDLL_WriteCmd(0xF1U);
	if (status != VOLOOP_OK) return status;

	status = VOLOOP_STM32_OLEDLL_WriteCmd(0xDAU);
	if (status != VOLOOP_OK) return status;
	status = VOLOOP_STM32_OLEDLL_WriteCmd(0x12U);
	if (status != VOLOOP_OK) return status;

	status = VOLOOP_STM32_OLEDLL_WriteCmd(0xDBU);
	if (status != VOLOOP_OK) return status;
	status = VOLOOP_STM32_OLEDLL_WriteCmd(0x40U);
	if (status != VOLOOP_OK) return status;

	status = VOLOOP_STM32_OLEDLL_WriteCmd(0x8DU);
	if (status != VOLOOP_OK) return status;
	status = VOLOOP_STM32_OLEDLL_WriteCmd(0x14U);
	if (status != VOLOOP_OK) return status;

	status = VOLOOP_STM32_OLEDLL_Clear();
	if (status != VOLOOP_OK) return status;

	return VOLOOP_STM32_OLEDLL_Start();
}

VOLOOP_StatusTypeDef VOLOOP_STM32_OLEDLL_Start(void) {
	if (s_oledCtx.isInitialized == 0U) {
		return VOLOOP_INVALID_STATE;
	}
	return VOLOOP_STM32_OLEDLL_WriteCmd(0xAFU);
}

VOLOOP_StatusTypeDef VOLOOP_STM32_OLEDLL_Stop(void) {
	if (s_oledCtx.isInitialized == 0U) {
		return VOLOOP_INVALID_STATE;
	}
	return VOLOOP_STM32_OLEDLL_WriteCmd(0xAEU);
}

VOLOOP_StatusTypeDef VOLOOP_STM32_OLEDLL_Clear(void) {
	VOLOOP_StatusTypeDef status;
	uint8_t zeroBuff[VOLOOP_STM32_OLEDLL_CLEAR_CHUNK] = {0};
	uint16_t loops = (uint16_t)(VOLOOP_STM32_OLEDLL_FRAME_BYTES / VOLOOP_STM32_OLEDLL_CLEAR_CHUNK);
	uint16_t i;

	if (s_oledCtx.isInitialized == 0U) {
		return VOLOOP_INVALID_STATE;
	}

	status = VOLOOP_STM32_OLEDLL_SetAddrWindow(0U,
											   (uint8_t)(VOLOOP_STM32_OLEDLL_WIDTH - 1U),
											   0U,
											   (uint8_t)(VOLOOP_STM32_OLEDLL_PAGE_COUNT - 1U));
	if (status != VOLOOP_OK) {
		return status;
	}

	for (i = 0U; i < loops; i++) {
		status = VOLOOP_STM32_OLEDLL_WriteDataStream(zeroBuff, VOLOOP_STM32_OLEDLL_CLEAR_CHUNK);
		if (status != VOLOOP_OK) {
			return status;
		}
	}

	return VOLOOP_OK;
}

VOLOOP_StatusTypeDef VOLOOP_STM32_OLEDLL_Refresh(const uint8_t* buffer, uint16_t length) {
	VOLOOP_StatusTypeDef status;

	if (s_oledCtx.isInitialized == 0U) {
		return VOLOOP_INVALID_STATE;
	}
	if (buffer == NULL || length != VOLOOP_STM32_OLEDLL_FRAME_BYTES) {
		return VOLOOP_INVALID_PARAM;
	}

	status = VOLOOP_STM32_OLEDLL_SetAddrWindow(0U,
											   (uint8_t)(VOLOOP_STM32_OLEDLL_WIDTH - 1U),
											   0U,
											   (uint8_t)(VOLOOP_STM32_OLEDLL_PAGE_COUNT - 1U));
	if (status != VOLOOP_OK) {
		return status;
	}

	return VOLOOP_STM32_OLEDLL_WriteDataStream(buffer, length);
}
