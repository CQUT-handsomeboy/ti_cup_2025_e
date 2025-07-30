#include "scanline.h"
#include "motor_pwm.h"

float Kp_sensor = 0.1, Ki_sensor = 0, Kd_sensor = 0;//pid弯道参数参数 
float sensor_bias = 0;
float sensor_bias_last = 0;
float P = 0, I = 0, D = 0, PID_value = 0;  //pid直道参数 
int decide;
extern 	uint8_t sensor[8];
float PWM_value_R,PWM_value_L;
void sensor_read(void)
{
	if((sensor[3] == 0)&&(sensor[4] == 0))// 00011000 
	{
		sensor_bias = 0;decide = 1;I=0;//积分项清零
	}
	else if((sensor[0] == 1)&&(sensor[1] == 1)&&(sensor[2] == 0)&&(sensor[3] == 0)&&(sensor[4] == 1)&&(sensor[5] == 1)&&(sensor[6] == 1)&&(sensor[7] == 1))// 00110000 相反的，为了观察
	{
		sensor_bias = -2;decide = 2;
	}
	else if((sensor[0] == 1)&&(sensor[1] == 1)&&(sensor[2] == 0)&&(sensor[3] == 1)&&(sensor[4] == 1)&&(sensor[5] == 1)&&(sensor[6] == 1)&&(sensor[7] == 1))// 00100000
	{
		sensor_bias = -3;decide = 2;
	}
	else if((sensor[0] == 1)&&(sensor[1] == 1)&&(sensor[2] == 1)&&(sensor[3] == 1)&&(sensor[4] == 0)&&(sensor[5] == 0)&&(sensor[6] == 1)&&(sensor[7] == 1))// 00001100 
	{
		sensor_bias = 2;decide = 2;
	}
	else if((sensor[0] == 1)&&(sensor[1] == 1)&&(sensor[2] == 1)&&(sensor[3] == 1)&&(sensor[4] == 1)&&(sensor[5] == 0)&&(sensor[6] == 1)&&(sensor[7] == 1))// 00000100 
	{
		sensor_bias = 3;decide = 2;
	}
	else if((sensor[0] == 1)&&(sensor[1] == 0)&&(sensor[2] == 0)&&(sensor[3] == 1)&&(sensor[4] == 1)&&(sensor[5] == 1)&&(sensor[6] == 1)&&(sensor[7] == 1))// 01100000
	{
		sensor_bias = -3.5;decide = 4;
	}
	else if((sensor[0] == 1)&&(sensor[1] == 0)&&(sensor[2] == 1)&&(sensor[3] == 1)&&(sensor[4] == 1)&&(sensor[5] == 1)&&(sensor[6] == 1)&&(sensor[7] == 1))// 01000000
	{
		sensor_bias = -4;decide = 4;
	}
	else if((sensor[0] == 1)&&(sensor[1] == 1)&&(sensor[2] == 1)&&(sensor[3] == 1)&&(sensor[4] == 1)&&(sensor[5] == 0)&&(sensor[6] == 0)&&(sensor[7] == 1))// 00000110
	{
		sensor_bias = 3.5;decide = 4;
	}
	else if((sensor[0] == 1)&&(sensor[1] == 1)&&(sensor[2] == 1)&&(sensor[3] == 1)&&(sensor[4] == 1)&&(sensor[5] == 1)&&(sensor[6] == 0)&&(sensor[7] == 1))// 00000010
	{
		sensor_bias = 4;decide = 4;
	}
	else if((sensor[0] == 1)&&(sensor[1] == 1)&&(sensor[2] == 1)&&(sensor[3] == 1)&&(sensor[4] == 1)&&(sensor[5] == 1)&&(sensor[6] == 0)&&(sensor[7] == 0))// 00000011
	{
		sensor_bias = 6.5;decide = 5;
	}
	else if((sensor[0] == 1)&&(sensor[1] == 1)&&(sensor[2] == 1)&&(sensor[3] == 1)&&(sensor[4] == 1)&&(sensor[5] == 1)&&(sensor[6] == 1)&&(sensor[7] == 0))// 00000001
	{
		sensor_bias = 10;decide = 5;
	}
	else if((sensor[0] == 0)&&(sensor[1] == 0)&&(sensor[2] == 1)&&(sensor[3] == 1)&&(sensor[4] == 1)&&(sensor[5] == 1)&&(sensor[6] == 1)&&(sensor[7] == 1))// 11000000
	{
		sensor_bias = -6.5;decide = 5;
	}
	else if((sensor[0] == 0)&&(sensor[1] == 1)&&(sensor[2] == 1)&&(sensor[3] == 1)&&(sensor[4] == 1)&&(sensor[5] == 1)&&(sensor[6] == 1)&&(sensor[7] == 1))// 10000000
	{
		sensor_bias = -10;decide = 5;
	}
	else if(((sensor[0] == 1)&&(sensor[1] == 1)&&(sensor[2] == 1)&&(sensor[3] == 1)&&(sensor[4] == 1)&&(sensor[5] == 1)&&(sensor[6] == 1)&&(sensor[7] == 1))||((sensor[0] == 0)&&(sensor[1] == 0)&&(sensor[2] == 0)&&(sensor[3] == 0)&&(sensor[4] == 0)&&(sensor[5] == 0)&&(sensor[6] == 0)&&(sensor[7] == 0)))
	{
		decide = 6; P = 0, I = 0, D = 0, PID_value = 0;
	}
}
void Sensor_pid(void)
{
	if(decide == 1){
//	    kongzhi(0, 0);
	}
	if(decide<6)
	{
		P = sensor_bias;
		I = I + sensor_bias;
		D = sensor_bias-sensor_bias_last;
		PID_value = Kp_sensor*P + Ki_sensor*I + Kd_sensor*D;
		sensor_bias_last = sensor_bias;
		
		//对积分值设置一个限制，防止积分值超标
//		if(I >=10)I = 10;
//		if(I <= -10)I = -10;
		
		PWM_value_R =  - PID_value;
		PWM_value_L =  + PID_value;
//		MotorSet(10+PWM_value_R,-(10+PWM_value_L));  // 修改，这里
    Motor_Pwm(-(50+PWM_value_L), 50+PWM_value_R);
	}
	else
	{
//		MotorSet(0,0);
		Motor_Pwm(0, 0);
	}
}
