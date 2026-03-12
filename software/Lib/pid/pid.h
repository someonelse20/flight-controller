#ifndef PID_H
#define PID_H

typedef struct pid_s {
	// Public variables
	const float DT;
	const float kP, kI, kD;
	const float gain;

	// Private internal variables
	float error;
	float previous_error;
	float proportional, integral, derivative;
} pid_t;

float loop(pid_t *pid, float procces_value, float set_point);

void reset(pid_t *pid);

#endif
