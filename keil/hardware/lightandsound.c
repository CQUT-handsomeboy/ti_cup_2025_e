#include "ti_msp_dl_config.h"
#include "Delay.h"
#include "lightandsound.h"

//声光控制代码start
void light_and_sound()
{
	DL_GPIO_setPins(GPIOA, Beep_Pin);
	DL_GPIO_setPins(GPIOA, Led_Pin);
	delay_ms(100);
	DL_GPIO_clearPins(GPIOA, Beep_Pin);
	DL_GPIO_clearPins(GPIOA,Led_Pin);
}
//声光控制代码end