/*
	ERMAO TEST 1
*/

#include "stdint.h"

float gray_calculate_bias(uint8_t sensor_data)
{
	static float prevError = 0;
	float error;
	switch(sensor_data)
	{
		// 0
		case 0b1111:
			error = prevError;
			break;
		// 1
		case 0b0111:
			error = -3;
			break;
		case 0b1011:
			error = -1;
			break;
		case 0b1101:
			error = 1;
			break;
		case 0b1110:
			error = 3;
			break;
		// 2
		case 0b0011:
			error = -2;
			break;
		case 0b1001:
			error = 0;
			break;
		case 0b1100:
			error = 2;
			break;
		// 3
		case 0b0001:
			error = -1;
			break;
		case 0b1000:
			error = 1;
			break;
		// 4
		case 0b0000:
			error = 0;
			break;
		// fuck
		default:
			error = prevError;
			break;
	}
	prevError = error;
	return error;
}