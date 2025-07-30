/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0L130X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0L130X

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define CPUCLK_FREQ                                                     32000000



/* Defines for PWM_0 */
#define PWM_0_INST                                                         TIMG4
#define PWM_0_INST_IRQHandler                                   TIMG4_IRQHandler
#define PWM_0_INST_INT_IRQN                                     (TIMG4_INT_IRQn)
#define PWM_0_INST_CLK_FREQ                                              2000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_0_C0_PORT                                                 GPIOA
#define GPIO_PWM_0_C0_PIN                                         DL_GPIO_PIN_17
#define GPIO_PWM_0_C0_IOMUX                                      (IOMUX_PINCM18)
#define GPIO_PWM_0_C0_IOMUX_FUNC                     IOMUX_PINCM18_PF_TIMG4_CCP0
#define GPIO_PWM_0_C0_IDX                                    DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_0_C1_PORT                                                 GPIOA
#define GPIO_PWM_0_C1_PIN                                         DL_GPIO_PIN_15
#define GPIO_PWM_0_C1_IOMUX                                      (IOMUX_PINCM16)
#define GPIO_PWM_0_C1_IOMUX_FUNC                     IOMUX_PINCM16_PF_TIMG4_CCP1
#define GPIO_PWM_0_C1_IDX                                    DL_TIMER_CC_1_INDEX



/* Defines for TIMER_0 */
#define TIMER_0_INST                                                     (TIMG0)
#define TIMER_0_INST_IRQHandler                                 TIMG0_IRQHandler
#define TIMER_0_INST_INT_IRQN                                   (TIMG0_INT_IRQn)
#define TIMER_0_INST_LOAD_VALUE                                         (31999U)




/* Defines for I2C_0 */
#define I2C_0_INST                                                          I2C0
#define I2C_0_INST_IRQHandler                                    I2C0_IRQHandler
#define I2C_0_INST_INT_IRQN                                        I2C0_INT_IRQn
#define I2C_0_BUS_SPEED_HZ                                                100000
#define GPIO_I2C_0_SDA_PORT                                                GPIOA
#define GPIO_I2C_0_SDA_PIN                                        DL_GPIO_PIN_10
#define GPIO_I2C_0_IOMUX_SDA                                     (IOMUX_PINCM11)
#define GPIO_I2C_0_IOMUX_SDA_FUNC                      IOMUX_PINCM11_PF_I2C0_SDA
#define GPIO_I2C_0_SCL_PORT                                                GPIOA
#define GPIO_I2C_0_SCL_PIN                                        DL_GPIO_PIN_11
#define GPIO_I2C_0_IOMUX_SCL                                     (IOMUX_PINCM12)
#define GPIO_I2C_0_IOMUX_SCL_FUNC                      IOMUX_PINCM12_PF_I2C0_SCL


/* Defines for UART_0 */
#define UART_0_INST                                                        UART1
#define UART_0_INST_IRQHandler                                  UART1_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART1_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOA
#define GPIO_UART_0_TX_PORT                                                GPIOA
#define GPIO_UART_0_RX_PIN                                        DL_GPIO_PIN_13
#define GPIO_UART_0_TX_PIN                                        DL_GPIO_PIN_14
#define GPIO_UART_0_IOMUX_RX                                     (IOMUX_PINCM14)
#define GPIO_UART_0_IOMUX_TX                                     (IOMUX_PINCM15)
#define GPIO_UART_0_IOMUX_RX_FUNC                      IOMUX_PINCM14_PF_UART1_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                      IOMUX_PINCM15_PF_UART1_TX
#define UART_0_BAUD_RATE                                                  (9600)
#define UART_0_IBRD_32_MHZ_9600_BAUD                                       (208)
#define UART_0_FBRD_32_MHZ_9600_BAUD                                        (21)





/* Port definition for Pin Group USER_GPIO */
#define USER_GPIO_PORT                                                   (GPIOA)

/* Defines for PWM_L0: GPIOA.24 with pinCMx 25 on package pin 28 */
#define USER_GPIO_PWM_L0_PIN                                    (DL_GPIO_PIN_24)
#define USER_GPIO_PWM_L0_IOMUX                                   (IOMUX_PINCM25)
/* Defines for PWM_L1: GPIOA.25 with pinCMx 26 on package pin 29 */
#define USER_GPIO_PWM_L1_PIN                                    (DL_GPIO_PIN_25)
#define USER_GPIO_PWM_L1_IOMUX                                   (IOMUX_PINCM26)
/* Defines for PULSE_A: GPIOA.22 with pinCMx 23 on package pin 26 */
// pins affected by this interrupt request:["PULSE_A","PULSE_B","PULSE_C","PULSE_D"]
#define USER_GPIO_INT_IRQN                                      (GPIOA_INT_IRQn)
#define USER_GPIO_INT_IIDX                      (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define USER_GPIO_PULSE_A_IIDX                              (DL_GPIO_IIDX_DIO22)
#define USER_GPIO_PULSE_A_PIN                                   (DL_GPIO_PIN_22)
#define USER_GPIO_PULSE_A_IOMUX                                  (IOMUX_PINCM23)
/* Defines for PULSE_B: GPIOA.23 with pinCMx 24 on package pin 27 */
#define USER_GPIO_PULSE_B_IIDX                              (DL_GPIO_IIDX_DIO23)
#define USER_GPIO_PULSE_B_PIN                                   (DL_GPIO_PIN_23)
#define USER_GPIO_PULSE_B_IOMUX                                  (IOMUX_PINCM24)
/* Defines for PWM_R0: GPIOA.4 with pinCMx 5 on package pin 8 */
#define USER_GPIO_PWM_R0_PIN                                     (DL_GPIO_PIN_4)
#define USER_GPIO_PWM_R0_IOMUX                                    (IOMUX_PINCM5)
/* Defines for PWM_R1: GPIOA.5 with pinCMx 6 on package pin 9 */
#define USER_GPIO_PWM_R1_PIN                                     (DL_GPIO_PIN_5)
#define USER_GPIO_PWM_R1_IOMUX                                    (IOMUX_PINCM6)
/* Defines for PULSE_C: GPIOA.6 with pinCMx 7 on package pin 10 */
#define USER_GPIO_PULSE_C_IIDX                               (DL_GPIO_IIDX_DIO6)
#define USER_GPIO_PULSE_C_PIN                                    (DL_GPIO_PIN_6)
#define USER_GPIO_PULSE_C_IOMUX                                   (IOMUX_PINCM7)
/* Defines for PULSE_D: GPIOA.7 with pinCMx 8 on package pin 11 */
#define USER_GPIO_PULSE_D_IIDX                               (DL_GPIO_IIDX_DIO7)
#define USER_GPIO_PULSE_D_PIN                                    (DL_GPIO_PIN_7)
#define USER_GPIO_PULSE_D_IOMUX                                   (IOMUX_PINCM8)
/* Port definition for Pin Group MPU6050 */
#define MPU6050_PORT                                                     (GPIOA)

/* Defines for SDA: GPIOA.0 with pinCMx 1 on package pin 1 */
#define MPU6050_SDA_PIN                                          (DL_GPIO_PIN_0)
#define MPU6050_SDA_IOMUX                                         (IOMUX_PINCM1)
/* Defines for SCL: GPIOA.1 with pinCMx 2 on package pin 2 */
#define MPU6050_SCL_PIN                                          (DL_GPIO_PIN_1)
#define MPU6050_SCL_IOMUX                                         (IOMUX_PINCM2)
/* Port definition for Pin Group GRAY */
#define GRAY_PORT                                                        (GPIOA)

/* Defines for DAT: GPIOA.2 with pinCMx 3 on package pin 6 */
#define GRAY_DAT_PIN                                             (DL_GPIO_PIN_2)
#define GRAY_DAT_IOMUX                                            (IOMUX_PINCM3)
/* Defines for CLK: GPIOA.3 with pinCMx 4 on package pin 7 */
#define GRAY_CLK_PIN                                             (DL_GPIO_PIN_3)
#define GRAY_CLK_IOMUX                                            (IOMUX_PINCM4)
/* Port definition for Pin Group lightandsound */
#define lightandsound_PORT                                               (GPIOA)

/* Defines for beep: GPIOA.12 with pinCMx 13 on package pin 16 */
#define lightandsound_beep_PIN                                  (DL_GPIO_PIN_12)
#define lightandsound_beep_IOMUX                                 (IOMUX_PINCM13)
/* Defines for led: GPIOA.16 with pinCMx 17 on package pin 20 */
#define lightandsound_led_PIN                                   (DL_GPIO_PIN_16)
#define lightandsound_led_IOMUX                                  (IOMUX_PINCM17)
/* Port definition for Pin Group KEY_Mode */
#define KEY_Mode_PORT                                                    (GPIOA)

/* Defines for Mode2: GPIOA.27 with pinCMx 28 on package pin 31 */
#define KEY_Mode_Mode2_PIN                                      (DL_GPIO_PIN_27)
#define KEY_Mode_Mode2_IOMUX                                     (IOMUX_PINCM28)
/* Defines for Mode3: GPIOA.26 with pinCMx 27 on package pin 30 */
#define KEY_Mode_Mode3_PIN                                      (DL_GPIO_PIN_26)
#define KEY_Mode_Mode3_IOMUX                                     (IOMUX_PINCM27)
/* Defines for Mode1: GPIOA.18 with pinCMx 19 on package pin 22 */
#define KEY_Mode_Mode1_PIN                                      (DL_GPIO_PIN_18)
#define KEY_Mode_Mode1_IOMUX                                     (IOMUX_PINCM19)

/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_0_init(void);
void SYSCFG_DL_TIMER_0_init(void);
void SYSCFG_DL_I2C_0_init(void);
void SYSCFG_DL_UART_0_init(void);



#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
