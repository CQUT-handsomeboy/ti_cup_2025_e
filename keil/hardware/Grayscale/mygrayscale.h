#ifndef __MYGRAYSCALE_H__
#define __MYGRAYSCALE_H__

#include "ti_msp_dl_config.h"
#include "Delay.h"

uint32_t updateRetWithGpioPin(GPIO_Regs* gpio, uint32_t pinMask, uint32_t ret, uint32_t i);
uint8_t gw_gray_serial_read();

#endif
