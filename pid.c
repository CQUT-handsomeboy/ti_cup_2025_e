/*
	ERMAO TEST 2
*/

#include "global.h"

float PID_calculate(PID * pid,float input,float target)
{
	float error = target - input;
	
	float P = pid->Kp *  error;// 比例
	
	pid->I += pid->Ki * error;// 积分累加
	// 积分限幅
	pid->I = pid->I > pid->I_max ? pid->I_max : pid->I;
	pid->I = pid->I < pid->I_min ? pid->I_min : pid->I;
	
	float D = pid->Kd * (error - pid->prev_error); // 微分
	
	
	pid->prev_error = error; // 误差传递
	
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