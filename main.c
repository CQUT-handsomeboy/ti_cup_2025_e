#include "ti_msp_dl_config.h"
#include "drv_oled.h"
#include "stdio.h"

#include "motor_pwm.h"
#include "ti/driverlib/dl_gpio.h"
#include "mpu6050.h"
#include "Kalman.h"

#include "string.h"
#include "stdlib.h"

#include "gw_grayscale_sensor.h"
#include "mygrayscale.h"
#include "scanline.h"

#include "global.h"
#include "Delay.h"
#include "lightandsound.h"

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

char pitchStr[1000], rollStr[1000], yawStr[1000];
char l_encoder[20], r_encoder[20];

uint8_t s1[32];
float Roll_value, Pitch_value, Yaw_value;

uint8_t sensor_data = 255;
extern float gray_calculate_bias(uint8_t sensor_data);


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

PID line_patrol_ultimate_pid = {
	.Kp = 40,
	.Ki = 0,
	.Kd = 0,
	.prev_error = 0,
	.I = 0,
	.I_max = 200,
	.I_min = -200,
	.max = 150,
	.min = -150,
};

PID speed2pwm = {
	.Kp = 2,
	.Ki = 1,
	.Kd = 1,
	.prev_error = 0,
	.I = 0,
	.I_max = 200,
	.I_min = -200,
	.max = 300,
	.min = -300,
};

PID straight_pid = {
	.Kp = 2,
	.Ki = 1,
	.Kd = 1,
	.prev_error = 0,
	.I = 0,
	.I_max = 200,
	.I_min = -200,
	.max = 300,
	.min = -300,
};

PID encoder_angle_pid = {
	.Kp = 1,
	.Ki = 0,
	.Kd = 0,
	.prev_error = 0,
	.I = 0,
	.I_max = 0,
	.I_min = 0,
	.max = 0,
	.min = 0,
};

extern float PID_calculate(PID *pid, float input, float target);
extern float PID_clear(PID *pid);
float base_ctrl_value = 550;
float base_speed = 250;

void display(int value)
{
	LCD_clear_L(3, 4);
	sprintf(s1, "%d",
			value);
	display_6_8_string(3, 4, s1);
}

void display_gray_sensor_data_but_not_read(void)
{
	LCD_clear_L(3, 4);
	sprintf(s1, "%d",
			sensor_data);
	display_6_8_string(3, 4, s1);
}

void display_encoder_count()
{
	LCD_clear_L(3, 1);
	sprintf(s1, "left:%d",
			l_encoder_l_count);
	display_6_8_string(3, 1, s1);

	LCD_clear_L(3, 2);
	sprintf(s1, "right:%d",
			r_encoder_l_count);
	display_6_8_string(3, 2, s1);
}

void display_speed()
{
	LCD_clear_L(3, 1);
	sprintf(s1, "left:%d",
			l_encoder_speed);
	display_6_8_string(3, 1, s1);

	LCD_clear_L(3, 2);
	sprintf(s1, "right:%d",
			r_encoder_speed);
	display_6_8_string(3, 2, s1);
}

#ifdef __KALMAN_H
void display_yaw(void)
{
	Kalman_GetEuler_Angle(&Roll_value, &Pitch_value, &Yaw_value);
	sprintf(yawStr, "Yam=%.2f", Yaw_value);
	display_6_8_string(4, 4, yawStr);
}
#endif

typedef enum
{
	LINE_PATROL,
	STRAIGHT,
	LINE_PATROL_ULTIMATE
} MODE;

MODE mode = 0xFF;

float speed_difference_ctrl_value;
float gray_bias;
float line_patrol_ctrl_value;
float line_patrol_weight;

float left_ctrl_value;
float right_ctrl_value;

void Handle_Mode()
{
	switch (mode)
	{

	case LINE_PATROL:
		sensor_data = gw_gray_serial_read();
		gray_bias = gray_calculate_bias(sensor_data);

		if (l_encoder_speed < 300)
		{
			line_patrol_weight = 10;
		}
		else
		{
			line_patrol_weight = 170;
		}

		speed_difference_ctrl_value = PID_calculate(&line_patrol_pid, l_encoder_speed, r_encoder_speed + gray_bias * line_patrol_weight);
		Motor_Pwm(base_ctrl_value + speed_difference_ctrl_value, base_ctrl_value);

		break;
	case STRAIGHT:
		speed_difference_ctrl_value = PID_calculate(&straight_pid, l_encoder_speed, r_encoder_speed);
		Motor_Pwm(speed_difference_ctrl_value + base_ctrl_value, base_ctrl_value);
		break;
	case LINE_PATROL_ULTIMATE:

		sensor_data = gw_gray_serial_read();
		gray_bias = gray_calculate_bias(sensor_data);

		left_ctrl_value = PID_calculate(&speed2pwm, l_encoder_speed, base_speed + gray_bias * 35);

		right_ctrl_value = PID_calculate(&speed2pwm, r_encoder_speed, base_speed - gray_bias * 35);

		Motor_Pwm(base_ctrl_value + left_ctrl_value, base_ctrl_value + right_ctrl_value);

		break;
	}
}

typedef enum
{
	LEFT,
	RIGHT
} MOTOR_DIRECTION;

void revolve_by_encoder(MOTOR_DIRECTION motor_direction, int delta_decoder)
{

	int start;
	switch (motor_direction)
	{
	case LEFT:
		start = l_encoder_l_count;
		while (l_encoder_l_count - start < delta_decoder)
		{
			Motor_Pwm(base_ctrl_value, 0);
		}
		break;
	case RIGHT:
		start = r_encoder_l_count;
		while (r_encoder_l_count - start < delta_decoder)
		{
			Motor_Pwm(0, base_ctrl_value);
		}
		break;
	}
	Motor_Pwm(0, 0);
}

#ifdef __KALMAN_H
void revolve_by_mpu6050_increasemental(int delta_yaw)
{

	float start_yaw;
	Kalman_GetEuler_Angle(&Roll_value, &Pitch_value, &start_yaw);
	display_yaw();
	if (delta_yaw > 0)
	{

		while (Yaw_value - start_yaw < delta_yaw)
		{
			display_yaw();
			Motor_Pwm(0, base_ctrl_value + 100);
		}
	}
	else
	{
		delta_yaw *= -1;

		while (start_yaw - Yaw_value < delta_yaw)
		{
			display_yaw();
			Motor_Pwm(base_ctrl_value + 100, 0);
		}
	}
	Motor_Pwm(0, 0);
}

void revolve_by_mpu6050_abusolute(int target_yaw)
{
	display_yaw();
	if (Yaw_value > target_yaw)
	{

		revolve_by_mpu6050_increasemental((int)(target_yaw - Yaw_value));
	}
	else
	{

		revolve_by_mpu6050_increasemental((int)(Yaw_value - target_yaw));
	}
}

#endif
void line_patrol()
{

	uint8_t pre_sensor_data;
	int start_l_encoder_count;

	mode = LINE_PATROL;
	PID_clear(&line_patrol_pid);

	start_l_encoder_count = l_encoder_l_count;

	while (1)
	{
		Handle_Mode();

		if (sensor_data == 255 && (l_encoder_l_count - start_l_encoder_count > 19000))
			break;

		pre_sensor_data = sensor_data;
	}

	int ctrl = gray_calculate_bias(pre_sensor_data);

	Motor_Pwm(0, 0);
	if (ctrl != 0)
	{

		if (ctrl > 0)
		{
			revolve_by_encoder(RIGHT, 550 * ctrl);
		}
		else
		{
			ctrl = (-1) * ctrl;
			revolve_by_encoder(LEFT, 550 * ctrl);
		}
	}
}

void line_patrol_ultimate()
{

	int start_r_encoder_count;

	mode = LINE_PATROL_ULTIMATE;
	PID_clear(&line_patrol_ultimate_pid);

	start_r_encoder_count = r_encoder_l_count;

	while (1)
	{
		Handle_Mode();

		if (sensor_data == 255 && (r_encoder_l_count - start_r_encoder_count > 19000))
			break;
	}

	Motor_Pwm(0, 0);
}

void Q1(int parking1, int parking2)
{

	if (parking1)
	{
		sensor_data = gw_gray_serial_read();
		while (sensor_data != 255)
		{

			sensor_data = gw_gray_serial_read();
			display_gray_sensor_data_but_not_read();
		}
	}

	mode = STRAIGHT;
	PID_clear(&straight_pid);

	while (1)
	{
		sensor_data = gw_gray_serial_read();
		display_gray_sensor_data_but_not_read();

		if (sensor_data != 255)
			break;

		Handle_Mode();
	}

	if (parking2)
	{
		Motor_Pwm(0, 0);
	}

	light_and_sound();
}

void Q2(void)
{
#if 0
	Q1(1,1); 
	delay_ms(100);
	line_patrol(); 
	light_and_sound(); 
	delay_ms(100);
	Q1(0,1); 
	delay_ms(100);
	line_patrol(); 
	light_and_sound();
#endif
	Q1(1, 1);
	delay_ms(100);
	line_patrol();
	light_and_sound();
	delay_ms(100);
	Q1(0, 1);
	delay_ms(100);
	line_patrol();
	light_and_sound();
}

void Q3_test(void)
{

	sensor_data = gw_gray_serial_read();
	display_gray_sensor_data_but_not_read();
	mode = STRAIGHT;
	while (1)
	{
		Handle_Mode();
		sensor_data = gw_gray_serial_read();
		display_gray_sensor_data_but_not_read();
		if (sensor_data == 0xFF)
			break;
	}
	Motor_Pwm(0, 0);
	delay_ms(100);
	revolve_by_encoder(LEFT, 2250);
	Q1(1, 1);
	light_and_sound();
	revolve_by_encoder(RIGHT, 4500);
	delay_ms(100);
	line_patrol_ultimate();
	delay_ms(100);
	light_and_sound();
	revolve_by_encoder(RIGHT, 1800);
	Q1(0, 1);
	revolve_by_encoder(LEFT, 4000);
	line_patrol();
	light_and_sound();
}

void Q4()
{

	Q1(1, 1);
	revolve_by_mpu6050_increasemental(60);
	line_patrol_ultimate();
	light_and_sound();
	revolve_by_mpu6050_increasemental(35);
	Q1(1, 1);
	light_and_sound();
	revolve_by_mpu6050_increasemental(-68);
	line_patrol();
	light_and_sound();
}

int main(void)
{
	SYSCFG_DL_init();

	oled_init();
	LCD_clear_L(0, 0);

#if 0
		DL_GPIO_togglePins(USER_GPIO_PORT, USER_GPIO_LED_PA0_PIN);  
		NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
		MPU6050_Init();
#endif
	NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
	NVIC_EnableIRQ(USER_GPIO_INT_IRQN);
	DL_TimerG_startCounter(TIMER_0_INST);
	uint8_t sensorStr[32];
#if 0
	while(1)
	{
		
		speed_difference_ctrl_value =  PID_calculate(&speed_difference,l_encoder_speed,r_encoder_speed + 100); 
		Motor_Pwm(base_ctrl_value + speed_difference_ctrl_value,base_ctrl_value);
		display_args(100);
	}
	
	while(1)
	{
		
		speed_difference_ctrl_value =  PID_calculate(&speed_difference,l_encoder_speed,r_encoder_speed + 100); 
		Motor_Pwm(base_ctrl_value + speed_difference_ctrl_value,base_ctrl_value);
		display_args(100);
	}
	
	while(1)
	{
		
		speed_difference_ctrl_value =  PID_calculate(&speed_difference,l_encoder_speed,r_encoder_speed + 200); 
		Motor_Pwm(base_ctrl_value + speed_difference_ctrl_value,base_ctrl_value);
		display_args(200);
	}
	
	while(1)
	{
		
		speed_difference_ctrl_value =  PID_calculate(&speed_difference,l_encoder_speed,r_encoder_speed - 100); 
		Motor_Pwm(base_ctrl_value + speed_difference_ctrl_value,base_ctrl_value);
		display_args(-100);
	}
	
	while(1)
	{
		
		speed_difference_ctrl_value =  PID_calculate(&speed_difference,l_encoder_speed,r_encoder_speed - 200); 
		Motor_Pwm(base_ctrl_value + speed_difference_ctrl_value,base_ctrl_value);
		display_args(- 200);
	}
#endif

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

#if 0
	PID left_speed2pwm = {
		.Kp = 2,
		.Ki= 1,
		.Kd = 1,
		.prev_error = 0,
		.I = 0,
		.I_max = 200,
		.I_min = -200,
		.max = 300,
		.min = -300,
	}; 
	
	float left_target_speed = 300;
	float left_ctrl;
	while(1)
	{
		left_ctrl = PID_calculate(&left_speed2pwm,l_encoder_speed,left_target_speed);
		Motor_Pwm(left_ctrl + base_ctrl_value,0);
		display_speed();
		display(left_ctrl);
	}

#endif


#if 0 
	while (1)
	{
		if (DL_GPIO_readPins(KEY_Mode_PORT, KEY_Mode_Mode1_PIN) > 0)
		{
			Q1(1, 1);
		}
		else if (DL_GPIO_readPins(KEY_Mode_PORT, KEY_Mode_Mode2_PIN) == 0)
		{
			Q2();
		}
		else if (DL_GPIO_readPins(KEY_Mode_PORT, KEY_Mode_Mode3_PIN) == 0)
		{
			Q4();
		}
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
