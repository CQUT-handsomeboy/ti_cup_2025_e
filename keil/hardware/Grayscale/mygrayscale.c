#include "mygrayscale.h"

uint32_t updateRetWithGpioPin(GPIO_Regs* gpio, uint32_t pinMask, uint32_t ret, uint32_t i)  
{  
    uint32_t pinState = DL_GPIO_readPins(gpio, pinMask);
    ret |= (pinState ? 1 : 0) << i; 
    return ret; 
}

uint8_t gw_gray_serial_read()
{
			uint8_t LineR1 = DL_GPIO_readPins(GRAY_PORT, GRAY_X4_PIN) > 0 ? 1 : 0;//读取右一
			uint8_t LineR2 = DL_GPIO_readPins(GRAY_PORT, GRAY_X3_PIN) > 0 ? 1 : 0;//读取右二
			uint8_t LineL2 = DL_GPIO_readPins(GRAY_PORT, GRAY_X1_PIN) > 0 ? 1 : 0;//读取左二
			uint8_t LineL1 = DL_GPIO_readPins(GRAY_PORT, GRAY_X2_PIN) > 0 ? 1 : 0;//读取左一
			return LineL1 << 3 | LineL2 << 2 | LineR2 << 1 | LineR1 << 0;
}