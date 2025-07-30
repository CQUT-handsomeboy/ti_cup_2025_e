/*
	ERMAO TEST 2
*/

#include "global.h"
#define I_MAX 300
#define MAX 500

float PID_calculate(PID * pid,float input,float target)
{
	float error = target - input;
	float P = pid->Kp * error;
	pid->I += pid->Ki * error;
	pid->I = pid->I > pid->I_max ? pid->I_max : pid->I;
	pid->I = pid->I < pid->I_min ? pid->I_min : pid->I;
	float D = pid->Kd * (error - pid->prev_error);
	pid->prev_error = error;
	float output = P + pid->I + D;
	output = output > pid->max ? pid->max : output;
	output = output < pid->min ? pid->min : output;
	return output;
}

float PID_clear(PID * pid)
{
	pid->I = 0;
	pid->prev_error = 0;
}

PID left_wheel_pid = {
	.Kp = 4,
	.Ki = 0,
	.Kd = 0,
	.prev_error = 0,
	.I = 0,
	.I_max = I_MAX,
	.I_min = -I_MAX,
	.max = MAX,
	.min = -MAX,
};

PID right_wheel_pid = {
	.Kp = 0,
	.Ki = 2,
	.Kd = 0.9,
	.prev_error = 0,
	.I = 0,
	.I_max = I_MAX,
	.I_min = -I_MAX,
	.max = MAX,
	.min = -MAX,
};