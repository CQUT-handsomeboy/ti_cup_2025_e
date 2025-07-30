#include "mygrayscale.h"

uint32_t updateRetWithGpioPin(GPIO_Regs* gpio, uint32_t pinMask, uint32_t ret, uint32_t i)  
{  
    uint32_t pinState = DL_GPIO_readPins(gpio, pinMask); // 读取引脚状态  
    ret |= (pinState ? 1 : 0) << i; // 如果引脚为高电平，则将1左移i位，并使用位或操作更新ret  
    return ret; // 返回更新后的ret值  
}  

uint8_t gw_gray_serial_read()
{
	uint8_t ret = 0;  // 要更新的变量 
  uint32_t i = 1;  // 左移的位数

	for (int i = 0; i < 8; ++i) {
		/* 输出时钟下降沿 */
		DL_GPIO_clearPins(GRAY_PORT, GRAY_CLK_PIN);
		delay_us(5);

		ret = updateRetWithGpioPin(GRAY_PORT, GRAY_DAT_PIN, ret, i); 

		/* 输出时钟上升沿,让传感器更新数据*/
		DL_GPIO_setPins(GRAY_PORT, GRAY_CLK_PIN);

		/* 主控频率高的需要给一点点延迟,延迟需要在5us左右 */
		delay_us(5);
	}

	return ret;
}