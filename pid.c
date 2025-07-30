/*
	ERMAO TEST 2
*/

#include "global.h"

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

PID line_patrol_pid = {
	.Kp = 4,
	.Ki = 2,
	.Kd = 0.9,
	.prev_error = 0,
	.I = 0,
	.I_max = 300,
	.I_min = -300,
	.max = 500,
	.min = -500,
};