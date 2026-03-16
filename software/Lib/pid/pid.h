#ifndef PID_H
#define PID_H

typedef struct {
	// Public variables
	const float DT;
	const float kP, kI, kD;
	const float gain;

	// Private internal variables
	float error;
	float previous_error;
	float proportional, integral, derivative;
} pid_control_t;

float loop_pid(pid_control_t *pid, float procces_value, float set_point);

void reset_pid(pid_control_t *pid);

#endif
