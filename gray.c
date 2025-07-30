/*
	ERMAO TEST 1
*/

#include "stdint.h"

/*
根据灰度二进制计算出偏差
*/
float gray_calculate_bias(uint8_t sensor_data)
{
	static float error = 0;
	switch(sensor_data)
	{
		// 0个0的情况
		case 0b11111111: // 没扫到黑线，不作任何处理，沿用上次结果
			break;
		// 1个0的情况
		case 0b01111111: 
			error = -3.5;
			break;
		case 0b10111111:
			error = -2.5;
			break;
		case 0b11011111:
			error = -1.5;
			break;
		case 0b11101111:
			error = -0.5;
			break;
		case 0b11110111:
			error = 0.5;
			break;
		case 0b11111011:
			error = 1.5;
			break;
		case 0b11111101:
			error = 2.5;
			break;
		case 0b11111110:
			error = 3.5;
			break;
		// 2个0的情况
		case 0b00111111:
			error = -3;
			break;
		case 0b10011111:
			error = -2;
			break;
		case 0b11001111:
			error = -1;
			break;
		case 0b11100111:
			error = 0;
			break;
		case 0b11110011:
			error = 1;
			break;
		case 0b11111001:
			error = 2;
			break;
		case 0b11111100:
			error = 3;
			break;
		// 3个0的情况
		case 0b00011111:
			error = -2.5;
			break;
		case 0b10001111:
			error = -1.5;
			break;
		case 0b11000111:
			error = -0.5;
			break;
		case 0b11100011:
			error = 0.5;
			break;
		case 0b11110001:
			error = 1.5;
			break;
		case 0b11111000:
			error = 2.5;
			break;
		// 4个0的情况
		case 0b00001111:
			error = -2;
			break;
		case 0b10000111:
			error = -1;
			break;
		case 0b11000011:
			error = 0;
			break;
		case 0b11100001:
			error = 1;
			break;
		case 0b11110000:
			error = 2;
			break;
		// 5个0的情况
		case 0b00000111:
			error = -1.5;
			break;
		case 0b10000011:
			error = -0.5;
			break;
		case 0b11000001:
			error = 0.5;
			break;
		case 0b11100000:
			error = 1.5;
			break;
		// 6个0的情况
		case 0b00000011:
			error = -1;
			break;
		case 0b10000001:
			error = 0;
			break;
		case 0b11000000:
			error = 1;
			break;
		// 7个0的情况
		case 0b00000001:
			error = -0.5;
			break;
		case 0b10000000:
			error = 0.5;
			break;
		// 8个0的情况
		case 0b00000000: // 全是黑线，走直线
			error = 0;
			break;
	}
	return error;
}

void gray_calculate_pid()
{
	
}