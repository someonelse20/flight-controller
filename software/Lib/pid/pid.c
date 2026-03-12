#include "pid.h"

float loop(pid_t *pid, float procces_value, float set_point) {
	float error = set_point - procces_value;

	float proportional = pid->kP * error;

	pid->integral = pid->integral + (pid->kI * error * pid->DT);

	float derivative = pid->kD * ((error - pid->previous_error) / pid->DT);

	pid->previous_error = error;

	float output = proportional + pid->integral + derivative;

	return output * pid->gain;
}

void reset(pid_t *pid) {
	pid->integral = 0;
	pid->previous_error = 0;
}

