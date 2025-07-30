#include "mygrayscale.h"

uint32_t updateRetWithGpioPin(GPIO_Regs* gpio, uint32_t pinMask, uint32_t ret, uint32_t i)  
{  
    uint32_t pinState = DL_GPIO_readPins(gpio, pinMask); // ��ȡ����״̬  
    ret |= (pinState ? 1 : 0) << i; // �������Ϊ�ߵ�ƽ����1����iλ����ʹ��λ���������ret  
    return ret; // ���ظ��º��retֵ  
}  

uint8_t gw_gray_serial_read()
{
	uint8_t ret = 0;  // Ҫ���µı��� 
  uint32_t i = 1;  // ���Ƶ�λ��

	for (int i = 0; i < 8; ++i) {
		/* ���ʱ���½��� */
		DL_GPIO_clearPins(GRAY_PORT, GRAY_CLK_PIN);
		delay_us(5);

		ret = updateRetWithGpioPin(GRAY_PORT, GRAY_DAT_PIN, ret, i); 

		/* ���ʱ��������,�ô�������������*/
		DL_GPIO_setPins(GRAY_PORT, GRAY_CLK_PIN);

		/* ����Ƶ�ʸߵ���Ҫ��һ����ӳ�,�ӳ���Ҫ��5us���� */
		delay_us(5);
	}

	return ret;
}