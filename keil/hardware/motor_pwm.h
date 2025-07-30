#ifndef __MOTOR_PWM_H
#define __MOTOR_PWM_H

#include "ti_msp_dl_config.h"

//void Motor_Start(GPTIMER_Regs *gptimer, int32_t value, DL_TIMER_CC_INDEX ccIndex);
void Motor_Start(GPTIMER_Regs *gptimer, int32_t value, DL_TIMER_CC_INDEX ccIndex);
void kongzhi(int l_pwm, int r_pwm);
void Motor_Pwm(int32_t l_value, int32_t r_value);
void Motor_Start_L(GPTIMER_Regs *gptimer, int32_t value, DL_TIMER_CC_INDEX ccIndex);
void Motor_Start_R(GPTIMER_Regs *gptimer, int32_t value, DL_TIMER_CC_INDEX ccIndex);
uint16_t Position_PID (int Encoder,int Target, float Position_KP, float Position_KI, float Position_KD);

#endif
