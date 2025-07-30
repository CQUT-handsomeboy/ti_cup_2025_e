#include "motor_pwm.h"
#include <stdlib.h>

extern int  l_encoder_speed;
extern int  r_encoder_speed;
int l_read_pwm=0, r_read_pwm=0;

void Motor_Start_L(GPTIMER_Regs *gptimer, int32_t value, DL_TIMER_CC_INDEX ccIndex)
{	
	DL_TimerG_startCounter(gptimer);
	DL_GPIO_setPins(USER_GPIO_PORT, USER_GPIO_PWM_L1_PIN);
	DL_GPIO_clearPins(USER_GPIO_PORT, USER_GPIO_PWM_L0_PIN);
	DL_TimerG_setCaptureCompareValue(gptimer, value, ccIndex);
}

void Motor_Start_R(GPTIMER_Regs *gptimer, int32_t value, DL_TIMER_CC_INDEX ccIndex)
{	
	DL_TimerG_startCounter(gptimer);
	DL_GPIO_setPins(USER_GPIO_PORT, USER_GPIO_PWM_R1_PIN);
	DL_GPIO_clearPins(USER_GPIO_PORT, USER_GPIO_PWM_R0_PIN);
	DL_TimerG_setCaptureCompareValue(gptimer, value, ccIndex);
}

void Motor_Pwm(int32_t l_value, int32_t r_value)
{	
	l_value = l_value > 1000 ? 1000 : l_value;
	r_value = r_value < 0 ? 0 : r_value;
	
	DL_TimerG_startCounter(PWM_0_INST);
	
	DL_GPIO_setPins(USER_GPIO_PORT, USER_GPIO_PWM_L1_PIN);
	DL_GPIO_clearPins(USER_GPIO_PORT, USER_GPIO_PWM_L0_PIN);
	DL_TimerG_setCaptureCompareValue(PWM_0_INST, l_value, GPIO_PWM_0_C0_IDX);
	// 电机右
	
	DL_GPIO_setPins(USER_GPIO_PORT, USER_GPIO_PWM_R1_PIN);
	DL_GPIO_clearPins(USER_GPIO_PORT, USER_GPIO_PWM_R0_PIN);
	DL_TimerG_setCaptureCompareValue(PWM_0_INST, r_value, GPIO_PWM_0_C1_IDX);
}
uint16_t Position_PID (int Encoder,int Target, float Position_KP, float Position_KI, float Position_KD)
{ 	
//	 float Position_KP=1.50,Position_KI=0.12,Position_KD=0;
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Target-Encoder;                                  //计算偏差
	 Integral_bias+=Bias;	                                 //求出偏差的积分
	 Pwm=Position_KP*Bias+Position_KI*Integral_bias+Position_KD*(Bias-Last_Bias);       //位置式PID控制器
	 Last_Bias=Bias;                                       //保存上一次偏差 
	 return Pwm;                                           //增量输出
}

void kongzhi(int l_pwm, int r_pwm)
{
//		l_read_pwm = Position_PID (l_encoder_speed,390+l_pwm, 1.55, 0.12, 0);
//		r_read_pwm = Position_PID (r_encoder_speed,390+r_pwm, 1.5, 0.11, 0);
		l_read_pwm = Position_PID (l_encoder_speed,390+l_pwm, 2.1, 0.01, 0.025);
		r_read_pwm = Position_PID (r_encoder_speed,390+r_pwm, 1.5, 0.11, 0);
		if( l_read_pwm>=0)
		{
						if(l_read_pwm>1000)
						{
										l_read_pwm=1000;
						}

						Motor_Start_L(PWM_0_INST, l_read_pwm, GPIO_PWM_0_C0_IDX) ;
		}
		if( r_read_pwm>=0)
		{
						if(r_read_pwm>1000)
						{
										r_read_pwm=1000;
						}

						Motor_Start_R(PWM_0_INST, r_read_pwm, GPIO_PWM_0_C1_IDX) ;
		}


//		else 
//		{
//		read_pwm=-read_pwm;


//						if(read_pwm>3200)
//						{
//						read_pwm=3200;
//						}

//				Motor_Back()    ;
//		}

}
