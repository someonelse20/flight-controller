#ifndef MOTOR_H
#define MOTOR_H

#include "stm32h7xx.h"
#include <stdint.h>

#define MAX_DUTY_CYCLE 100
#define MIN_DUTY_CYCLE 0

typedef struct motor_s {
	const uint32_t channel;
	TIM_HandleTypeDef *htim;
	int min_speed;
	int max_speed;
} motor_t;

int start_motor(motor_t motor);

int stop_motor(motor_t motor);

// Speed is a percentage of 0 (off) to 100 (fully on) between the minimum value and the maximum.
int set_motor_speed(motor_t motor, int speed);

int set_true_motor_speed(motor_t motor, int speed); // Raw pwm values to write to the motor.

int calibrate_motor(motor_t *motor);

int write_motor_config();

#endif
