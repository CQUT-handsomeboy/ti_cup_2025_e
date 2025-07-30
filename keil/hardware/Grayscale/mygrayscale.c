#include "mygrayscale.h"

uint32_t updateRetWithGpioPin(GPIO_Regs* gpio, uint32_t pinMask, uint32_t ret, uint32_t i)  
{  
    uint32_t pinState = DL_GPIO_readPins(gpio, pinMask);
    ret |= (pinState ? 1 : 0) << i; 
    return ret; 
}

uint8_t gw_gray_serial_read()
{
	uint8_t ret = 0;
  uint32_t i = 1;
	for (int i = 0; i < 8; ++i) {
		DL_GPIO_clearPins(GRAY_PORT, GRAY_CLK_PIN);
		delay_us(5);
		ret = updateRetWithGpioPin(GRAY_PORT, GRAY_DAT_PIN, ret, i); 
		DL_GPIO_setPins(GRAY_PORT, GRAY_CLK_PIN);
		delay_us(5);
	}
	return ret;
}