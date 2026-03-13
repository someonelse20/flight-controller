#include "motor.h"
#include "stm32h723xx.h"
#include "stm32h7xx_hal_tim.h"

int start_motor(motor_t motor) {
	return HAL_TIM_PWM_Start(motor.htim, motor.channel);
}

int stop_motor(motor_t motor) {
	return HAL_TIM_PWM_Stop(motor.htim, motor.channel);
}

int set_motor_speed(motor_t motor, int speed) {
	if (motor.channel == 1) {
		motor.htim->Instance->CCR3 = speed;
	} else if (motor.channel == 2) {
		motor.htim->Instance->CCR3 = speed;
	} else if (motor.channel == 3){
		motor.htim->Instance->CCR3 = speed;
	} else if (motor.channel == 4) {
		motor.htim->Instance->CCR4 = speed;
	} else { // Incorrect channel value
		return 1;
	}

	return 0;
}

