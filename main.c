#include "ti_msp_dl_config.h"

#include "stdio.h"

#include "motor_pwm.h"
#include "ti/driverlib/dl_gpio.h"
#include "mpu6050.h"
#include "Kalman.h"

#include "string.h"
#include "stdlib.h"
#include "drv_oled.h"
#include "gw_grayscale_sensor.h"
#include "mygrayscale.h"
#include "scanline.h"

#include "global.h"
#include "Delay.h"
#include "lightandsound.h"

// BASE_CTRL represents the base speed. 
// In order to achieve a turn, 
// the difference is added or subtracted based on this speed.

#define BASE_CTRL 550
#define BASE_SPEED 550
#define INC_SPEED	200

void usart0_send_bytes(unsigned char *buf, int len)
{
	while (len--)
	{
		DL_UART_Main_transmitDataBlocking(UART_0_INST, *buf);
		buf++;
	}
}

uint32_t l_encoder_l_count = 0;
uint32_t l_last_encoder_count = 0;
uint32_t l_encoder_speed = 0;
uint32_t r_encoder_l_count = 0;
uint32_t r_last_encoder_count = 0;
uint32_t r_encoder_speed = 0;

#define DELAY (3200000)
uint8_t sensor[8];
char l_encoder[20], r_encoder[20];
uint8_t sensor_data = 255;
extern float gray_calculate_bias(uint8_t sensor_data);
extern float PID_calculate(PID *pid, float input, float target);
extern float PID_clear(PID *pid);

// The only functions we need are two: 
// a tracking function, where the tracking module stops 
// when it can't find the black line; 
// and a quarter-angle turn function, 
// where the angle is controlled by an encoder.

float speed_difference_ctrl_value,
		gray_bias,
		line_patrol_ctrl_value,
		line_patrol_weight,
		left_ctrl_value,
		right_ctrl_value;

typedef enum
{
	LEFT,
	RIGHT
} MOTOR_DIRECTION;
// This function controls the motor 
// to rotate a certain angle by combining the encoder readings. 
// It only needs to measure the number of pulses 
// for a right-angle turn.
void revolve_by_encoder(MOTOR_DIRECTION motor_direction, int delta_decoder)
{
	int start;
	switch (motor_direction)
	{
	case LEFT:
		start = l_encoder_l_count;
		while (l_encoder_l_count - start < delta_decoder)
		{
			Motor_Pwm(BASE_CTRL, 0);
		}
		break;
	case RIGHT:
		start = r_encoder_l_count;
		while (r_encoder_l_count - start < delta_decoder)
		{
			Motor_Pwm(0, BASE_CTRL);
		}
		break;
	}
	Motor_Pwm(0, 0);
}

char s1[16];
extern PID left_wheel_pid,right_wheel_pid;
void line_patrol()
{
	while (1) 
	{
		sensor_data = gw_gray_serial_read();
		gray_bias = gray_calculate_bias(sensor_data);
		// Turn right and the left wheel will go faster
		// Be positive if you need to turn right

		// Note that both should be 
		// positive or negative numbers
		float inc_l = PID_calculate(&left_wheel_pid,
			l_encoder_speed,
			BASE_SPEED + gray_bias * INC_SPEED
		);
		float inc_r = PID_calculate(&right_wheel_pid,
			r_encoder_speed,
			BASE_SPEED - gray_bias * INC_SPEED
		);
		Motor_Pwm(BASE_CTRL + inc_l, BASE_CTRL + inc_r);
		//		sprintf(s1,"%06.2f",gray_bias);
		//		LCD_clear_L(1, 1);
		//		display_6_8_string(1,1,s1);
		delay_ms(100);
	}
}

extern void float_items_screen_show(
		char* title,
		char** item_titles,
		float* item_value
);

int main(void)
{
	SYSCFG_DL_init();

	oled_init();
	LCD_clear_L(0, 0);

	NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
	NVIC_EnableIRQ(USER_GPIO_INT_IRQN);
	DL_TimerG_startCounter(TIMER_0_INST);

	/*
					   _ooOoo_
					  o8888888o
					  88" . "88
					  (| -_- |)
					  O\  =  /O
				   ____/`---'\____
				 .'  \\|     |
				/  \\|||  :  |||
			   /  _||||| -:- |||||-  \
			   |   | \\\  -
			   | \_|  ''\---/''  |   |
			   \  .-\__  `-`  ___/-. /
			 ___`. .'  /--.--\  `. . __
		  ."" '<  `.___\_<|>_/___.'  >'"".
		 | | :  `- \`.;`\ _ /`;.`/ - ` : | |
		 \  \ `-.   \_ __\ /__ _/   .-` /  /
	======`-.____`-.___\_____/___.-`____.-'======
					   `=---='
	^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	*/
	while(1)
	{
		line_patrol();
		delay_ms(100);
	}
#if 0 
// Mode Switching Reference
	while (1) {
		if (DL_GPIO_readPins(KEY_Mode_PORT, KEY_Mode_Mode1_PIN) > 0) Q1(1, 1);
		else if (DL_GPIO_readPins(KEY_Mode_PORT, KEY_Mode_Mode2_PIN) == 0) Q2();
		else if (DL_GPIO_readPins(KEY_Mode_PORT, KEY_Mode_Mode3_PIN) == 0) Q4();
	}
#endif
}

void TIMER_0_INST_IRQHandler(void)
{
	uint32_t l_current_count = l_encoder_l_count;
	uint32_t l_steps_this_second = l_current_count - l_last_encoder_count;
	l_last_encoder_count = l_current_count;

	l_encoder_speed = l_steps_this_second;

	uint32_t r_current_count = r_encoder_l_count;
	uint32_t r_steps_this_second = r_current_count - r_last_encoder_count;
	r_last_encoder_count = r_current_count;

	r_encoder_speed = r_steps_this_second;
}

void GROUP1_IRQHandler(void)
{
	uint8_t x_l, y_l, z_l, sum_l;
	uint8_t x_r, y_r, z_r, sum_r;
	switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1))
	{
	case USER_GPIO_INT_IIDX:

		if (DL_GPIO_getEnabledInterruptStatus(GPIOA, USER_GPIO_PULSE_A_PIN))
		{
			x_l = 1;

			if (DL_GPIO_readPins(GPIOA, USER_GPIO_PULSE_A_PIN))
				y_l = 1;
			else
				y_l = 0;

			if (DL_GPIO_readPins(GPIOA, USER_GPIO_PULSE_B_PIN))
				z_l = 1;
			else
				z_l = 0;

			sum_l = x_l + y_l + z_l;
			if (sum_l == 0 || sum_l == 2)
				l_encoder_l_count++;
			else
				l_encoder_l_count--;

			DL_GPIO_clearInterruptStatus(GPIOA, USER_GPIO_PULSE_A_PIN);
		}

		if (DL_GPIO_getEnabledInterruptStatus(GPIOA, USER_GPIO_PULSE_B_PIN))
		{
			x_l = 0;

			if (DL_GPIO_readPins(GPIOA, USER_GPIO_PULSE_A_PIN))
				y_l = 1;
			else
				y_l = 0;

			if (DL_GPIO_readPins(GPIOA, USER_GPIO_PULSE_B_PIN))
				z_l = 1;
			else
				z_l = 0;

			sum_l = x_l + y_l + z_l;
			if (sum_l == 0 || sum_l == 2)
				l_encoder_l_count++;
			else
				l_encoder_l_count--;

			DL_GPIO_clearInterruptStatus(GPIOA, USER_GPIO_PULSE_B_PIN);
		}

		if (DL_GPIO_getEnabledInterruptStatus(GPIOA, USER_GPIO_PULSE_C_PIN))
		{
			x_r = 1;

			if (DL_GPIO_readPins(GPIOA, USER_GPIO_PULSE_C_PIN))
				y_r = 1;
			else
				y_r = 0;

			if (DL_GPIO_readPins(GPIOA, USER_GPIO_PULSE_D_PIN))
				z_r = 1;
			else
				z_r = 0;

			sum_r = x_r + y_r + z_r;
			if (sum_r == 0 || sum_r == 2)
				r_encoder_l_count++;
			else
				r_encoder_l_count--;

			DL_GPIO_clearInterruptStatus(GPIOA, USER_GPIO_PULSE_C_PIN);
		}

		if (DL_GPIO_getEnabledInterruptStatus(GPIOA, USER_GPIO_PULSE_D_PIN))
		{
			x_r = 0;

			if (DL_GPIO_readPins(GPIOA, USER_GPIO_PULSE_C_PIN))
				y_r = 1;
			else
				y_r = 0;

			if (DL_GPIO_readPins(GPIOA, USER_GPIO_PULSE_D_PIN))
				z_r = 1;
			else
				z_r = 0;

			sum_r = x_r + y_r + z_r;
			if (sum_r == 0 || sum_r == 2)
				r_encoder_l_count++;
			else
				r_encoder_l_count--;

			DL_GPIO_clearInterruptStatus(GPIOA, USER_GPIO_PULSE_D_PIN);
		}

		break;
	}
}
