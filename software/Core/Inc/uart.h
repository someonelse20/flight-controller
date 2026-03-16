#ifndef UART_H
#define UART_H

#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_uart.h"
#include <stdint.h>
#include <sys/types.h>

typedef struct {
	float throttle;
	float set_x;
	float set_y;
	float set_z;
} controls_t;

typedef struct {
	double latitude;
	double longitude;
	uint16_t altidude;
	uint16_t heading;
	uint16_t speed;
} telemetry_t;

extern controls_t controls;
extern telemetry_t telemetry;

extern UART_HandleTypeDef transceiver_h;

uint8_t uart_scan(UART_HandleTypeDef interfaces[], uint32_t size);
uint8_t transmit_telemetry();

#endif
