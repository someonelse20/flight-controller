#include "uart.h"
#include "main.h"
#include "sdmmc.h"
#include "stm32h7xx_hal_def.h"
#include "stm32h7xx_hal_uart.h"
#include "transceiver_reg.h"
#include <stdint.h>
#include <stdbool.h>
#include <sys/types.h>

controls_t controls;
telemetry_t telemetry;

UART_HandleTypeDef transceiver_h;

const uint8_t TRANSCEIVER_ID = 126;

static uint8_t control_buf[sizeof(controls)];

static bool device_found = false;

uint8_t uart_scan(UART_HandleTypeDef uart_interfaces[], uint32_t size) {
	uint8_t who_am_i_reg = 0x00;
	uint8_t buf;

	HAL_StatusTypeDef status;

	for (int i = 0; i < size; i++) {
		status = HAL_UART_Transmit(&uart_interfaces[i], &who_am_i_reg, 1, HAL_TIMEOUT);

		if (status != HAL_OK) {
			continue;
		}

		HAL_UART_Receive(&uart_interfaces[i], &buf, 1, HAL_TIMEOUT);

		if (buf == TRANSCEIVER_ID) {
			sd_log("INFO", "Transciever", "Transceiver found");
			transceiver_h = uart_interfaces[i];
			device_found = true;
		}
	}

	if (!device_found) {
		sd_log("WARNING", "UART", "No UART device found, is it plugged in?");
		return 1;
	}

	return 0;
}

uint8_t transmit_telemetry() {
	uint8_t reg = TX_REG;
	uint8_t size = sizeof(telemetry);

	if (HAL_UART_Transmit(&transceiver_h, &reg, 1, HAL_TIMEOUT)) {
		return 1;
	}

	if (HAL_UART_Transmit(&transceiver_h, &size, 1, HAL_TIMEOUT)) {
		return 1;
	}

	if (HAL_UART_Transmit(&transceiver_h, (uint8_t *)&telemetry, size, HAL_TIMEOUT) != HAL_OK) {
		return 1;
	}

	return 0;
}

