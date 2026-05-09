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
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

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



#define CPUCLK_FREQ                                                     80000000
/* Defines for SYSPLL_ERR_01 Workaround */
/* Represent 1.000 as 1000 */
#define FLOAT_TO_INT_SCALE                                               (1000U)
#define FCC_EXPECTED_RATIO                                                  2500
#define FCC_UPPER_BOUND                       (FCC_EXPECTED_RATIO * (1 + 0.003))
#define FCC_LOWER_BOUND                       (FCC_EXPECTED_RATIO * (1 - 0.003))

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);


/* Defines for PWM_MOTOR */
#define PWM_MOTOR_INST                                                     TIMA0
#define PWM_MOTOR_INST_IRQHandler                               TIMA0_IRQHandler
#define PWM_MOTOR_INST_INT_IRQN                                 (TIMA0_INT_IRQn)
#define PWM_MOTOR_INST_CLK_FREQ                                         80000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_MOTOR_C0_PORT                                             GPIOB
#define GPIO_PWM_MOTOR_C0_PIN                                     DL_GPIO_PIN_14
#define GPIO_PWM_MOTOR_C0_IOMUX                                  (IOMUX_PINCM31)
#define GPIO_PWM_MOTOR_C0_IOMUX_FUNC                 IOMUX_PINCM31_PF_TIMA0_CCP0
#define GPIO_PWM_MOTOR_C0_IDX                                DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_MOTOR_C1_PORT                                             GPIOA
#define GPIO_PWM_MOTOR_C1_PIN                                      DL_GPIO_PIN_7
#define GPIO_PWM_MOTOR_C1_IOMUX                                  (IOMUX_PINCM14)
#define GPIO_PWM_MOTOR_C1_IOMUX_FUNC                 IOMUX_PINCM14_PF_TIMA0_CCP1
#define GPIO_PWM_MOTOR_C1_IDX                                DL_TIMER_CC_1_INDEX



/* Defines for TIMER_CALC */
#define TIMER_CALC_INST                                                  (TIMA1)
#define TIMER_CALC_INST_IRQHandler                              TIMA1_IRQHandler
#define TIMER_CALC_INST_INT_IRQN                                (TIMA1_INT_IRQn)
#define TIMER_CALC_INST_LOAD_VALUE                                       (3124U)




/* Defines for I2C_MPU6050 */
#define I2C_MPU6050_INST                                                    I2C1
#define I2C_MPU6050_INST_IRQHandler                              I2C1_IRQHandler
#define I2C_MPU6050_INST_INT_IRQN                                  I2C1_INT_IRQn
#define I2C_MPU6050_BUS_SPEED_HZ                                          400000
#define GPIO_I2C_MPU6050_SDA_PORT                                          GPIOB
#define GPIO_I2C_MPU6050_SDA_PIN                                   DL_GPIO_PIN_3
#define GPIO_I2C_MPU6050_IOMUX_SDA                               (IOMUX_PINCM16)
#define GPIO_I2C_MPU6050_IOMUX_SDA_FUNC                IOMUX_PINCM16_PF_I2C1_SDA
#define GPIO_I2C_MPU6050_SCL_PORT                                          GPIOB
#define GPIO_I2C_MPU6050_SCL_PIN                                   DL_GPIO_PIN_2
#define GPIO_I2C_MPU6050_IOMUX_SCL                               (IOMUX_PINCM15)
#define GPIO_I2C_MPU6050_IOMUX_SCL_FUNC                IOMUX_PINCM15_PF_I2C1_SCL

/* Defines for I2C_OLED */
#define I2C_OLED_INST                                                       I2C0
#define I2C_OLED_INST_IRQHandler                                 I2C0_IRQHandler
#define I2C_OLED_INST_INT_IRQN                                     I2C0_INT_IRQn
#define I2C_OLED_BUS_SPEED_HZ                                             400000
#define GPIO_I2C_OLED_SDA_PORT                                             GPIOA
#define GPIO_I2C_OLED_SDA_PIN                                      DL_GPIO_PIN_0
#define GPIO_I2C_OLED_IOMUX_SDA                                   (IOMUX_PINCM1)
#define GPIO_I2C_OLED_IOMUX_SDA_FUNC                    IOMUX_PINCM1_PF_I2C0_SDA
#define GPIO_I2C_OLED_SCL_PORT                                             GPIOA
#define GPIO_I2C_OLED_SCL_PIN                                      DL_GPIO_PIN_1
#define GPIO_I2C_OLED_IOMUX_SCL                                   (IOMUX_PINCM2)
#define GPIO_I2C_OLED_IOMUX_SCL_FUNC                    IOMUX_PINCM2_PF_I2C0_SCL


/* Defines for UART0 */
#define UART0_INST                                                         UART0
#define UART0_INST_FREQUENCY                                            40000000
#define UART0_INST_IRQHandler                                   UART0_IRQHandler
#define UART0_INST_INT_IRQN                                       UART0_INT_IRQn
#define GPIO_UART0_RX_PORT                                                 GPIOA
#define GPIO_UART0_TX_PORT                                                 GPIOA
#define GPIO_UART0_RX_PIN                                         DL_GPIO_PIN_11
#define GPIO_UART0_TX_PIN                                         DL_GPIO_PIN_10
#define GPIO_UART0_IOMUX_RX                                      (IOMUX_PINCM22)
#define GPIO_UART0_IOMUX_TX                                      (IOMUX_PINCM21)
#define GPIO_UART0_IOMUX_RX_FUNC                       IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART0_IOMUX_TX_FUNC                       IOMUX_PINCM21_PF_UART0_TX
#define UART0_BAUD_RATE                                                 (115200)
#define UART0_IBRD_40_MHZ_115200_BAUD                                       (21)
#define UART0_FBRD_40_MHZ_115200_BAUD                                       (45)
/* Defines for UART_BT */
#define UART_BT_INST                                                       UART1
#define UART_BT_INST_FREQUENCY                                          40000000
#define UART_BT_INST_IRQHandler                                 UART1_IRQHandler
#define UART_BT_INST_INT_IRQN                                     UART1_INT_IRQn
#define GPIO_UART_BT_RX_PORT                                               GPIOA
#define GPIO_UART_BT_TX_PORT                                               GPIOA
#define GPIO_UART_BT_RX_PIN                                        DL_GPIO_PIN_9
#define GPIO_UART_BT_TX_PIN                                        DL_GPIO_PIN_8
#define GPIO_UART_BT_IOMUX_RX                                    (IOMUX_PINCM20)
#define GPIO_UART_BT_IOMUX_TX                                    (IOMUX_PINCM19)
#define GPIO_UART_BT_IOMUX_RX_FUNC                     IOMUX_PINCM20_PF_UART1_RX
#define GPIO_UART_BT_IOMUX_TX_FUNC                     IOMUX_PINCM19_PF_UART1_TX
#define UART_BT_BAUD_RATE                                                 (9600)
#define UART_BT_IBRD_40_MHZ_9600_BAUD                                      (260)
#define UART_BT_FBRD_40_MHZ_9600_BAUD                                       (27)





/* Port definition for Pin Group GPIO_MOTOR */
#define GPIO_MOTOR_PORT                                                  (GPIOB)

/* Defines for PIN_AIN1: GPIOB.9 with pinCMx 26 on package pin 61 */
#define GPIO_MOTOR_PIN_AIN1_PIN                                  (DL_GPIO_PIN_9)
#define GPIO_MOTOR_PIN_AIN1_IOMUX                                (IOMUX_PINCM26)
/* Defines for PIN_AIN2: GPIOB.10 with pinCMx 27 on package pin 62 */
#define GPIO_MOTOR_PIN_AIN2_PIN                                 (DL_GPIO_PIN_10)
#define GPIO_MOTOR_PIN_AIN2_IOMUX                                (IOMUX_PINCM27)
/* Defines for PIN_BIN1: GPIOB.7 with pinCMx 24 on package pin 59 */
#define GPIO_MOTOR_PIN_BIN1_PIN                                  (DL_GPIO_PIN_7)
#define GPIO_MOTOR_PIN_BIN1_IOMUX                                (IOMUX_PINCM24)
/* Defines for PIN_BIN2: GPIOB.6 with pinCMx 23 on package pin 58 */
#define GPIO_MOTOR_PIN_BIN2_PIN                                  (DL_GPIO_PIN_6)
#define GPIO_MOTOR_PIN_BIN2_IOMUX                                (IOMUX_PINCM23)
/* Port definition for Pin Group GPIO_ENCODER */
#define GPIO_ENCODER_PORT                                                (GPIOB)

/* Defines for PIN_A1: GPIOB.11 with pinCMx 28 on package pin 63 */
// pins affected by this interrupt request:["PIN_A1","PIN_A2","PIN_B1","PIN_B2"]
#define GPIO_ENCODER_INT_IRQN                                   (GPIOB_INT_IRQn)
#define GPIO_ENCODER_INT_IIDX                   (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define GPIO_ENCODER_PIN_A1_IIDX                            (DL_GPIO_IIDX_DIO11)
#define GPIO_ENCODER_PIN_A1_PIN                                 (DL_GPIO_PIN_11)
#define GPIO_ENCODER_PIN_A1_IOMUX                                (IOMUX_PINCM28)
/* Defines for PIN_A2: GPIOB.4 with pinCMx 17 on package pin 52 */
#define GPIO_ENCODER_PIN_A2_IIDX                             (DL_GPIO_IIDX_DIO4)
#define GPIO_ENCODER_PIN_A2_PIN                                  (DL_GPIO_PIN_4)
#define GPIO_ENCODER_PIN_A2_IOMUX                                (IOMUX_PINCM17)
/* Defines for PIN_B1: GPIOB.12 with pinCMx 29 on package pin 64 */
#define GPIO_ENCODER_PIN_B1_IIDX                            (DL_GPIO_IIDX_DIO12)
#define GPIO_ENCODER_PIN_B1_PIN                                 (DL_GPIO_PIN_12)
#define GPIO_ENCODER_PIN_B1_IOMUX                                (IOMUX_PINCM29)
/* Defines for PIN_B2: GPIOB.5 with pinCMx 18 on package pin 53 */
#define GPIO_ENCODER_PIN_B2_IIDX                             (DL_GPIO_IIDX_DIO5)
#define GPIO_ENCODER_PIN_B2_PIN                                  (DL_GPIO_PIN_5)
#define GPIO_ENCODER_PIN_B2_IOMUX                                (IOMUX_PINCM18)
/* Defines for PIN_OUT1: GPIOB.19 with pinCMx 45 on package pin 16 */
#define GPIO_TRM_PIN_OUT1_PORT                                           (GPIOB)
#define GPIO_TRM_PIN_OUT1_PIN                                   (DL_GPIO_PIN_19)
#define GPIO_TRM_PIN_OUT1_IOMUX                                  (IOMUX_PINCM45)
/* Defines for PIN_OUT2: GPIOB.17 with pinCMx 43 on package pin 14 */
#define GPIO_TRM_PIN_OUT2_PORT                                           (GPIOB)
#define GPIO_TRM_PIN_OUT2_PIN                                   (DL_GPIO_PIN_17)
#define GPIO_TRM_PIN_OUT2_IOMUX                                  (IOMUX_PINCM43)
/* Defines for PIN_OUT3: GPIOA.16 with pinCMx 38 on package pin 9 */
#define GPIO_TRM_PIN_OUT3_PORT                                           (GPIOA)
#define GPIO_TRM_PIN_OUT3_PIN                                   (DL_GPIO_PIN_16)
#define GPIO_TRM_PIN_OUT3_IOMUX                                  (IOMUX_PINCM38)
/* Defines for PIN_OUT4: GPIOA.14 with pinCMx 36 on package pin 7 */
#define GPIO_TRM_PIN_OUT4_PORT                                           (GPIOA)
#define GPIO_TRM_PIN_OUT4_PIN                                   (DL_GPIO_PIN_14)
#define GPIO_TRM_PIN_OUT4_IOMUX                                  (IOMUX_PINCM36)
/* Defines for PIN_OUT5: GPIOB.20 with pinCMx 48 on package pin 19 */
#define GPIO_TRM_PIN_OUT5_PORT                                           (GPIOB)
#define GPIO_TRM_PIN_OUT5_PIN                                   (DL_GPIO_PIN_20)
#define GPIO_TRM_PIN_OUT5_IOMUX                                  (IOMUX_PINCM48)
/* Defines for PIN_OUT6: GPIOB.25 with pinCMx 56 on package pin 27 */
#define GPIO_TRM_PIN_OUT6_PORT                                           (GPIOB)
#define GPIO_TRM_PIN_OUT6_PIN                                   (DL_GPIO_PIN_25)
#define GPIO_TRM_PIN_OUT6_IOMUX                                  (IOMUX_PINCM56)
/* Defines for PIN_OUT7: GPIOA.25 with pinCMx 55 on package pin 26 */
#define GPIO_TRM_PIN_OUT7_PORT                                           (GPIOA)
#define GPIO_TRM_PIN_OUT7_PIN                                   (DL_GPIO_PIN_25)
#define GPIO_TRM_PIN_OUT7_IOMUX                                  (IOMUX_PINCM55)
/* Defines for PIN_1: GPIOA.15 with pinCMx 37 on package pin 8 */
#define GPIO_LED_PIN_1_PORT                                              (GPIOA)
#define GPIO_LED_PIN_1_PIN                                      (DL_GPIO_PIN_15)
#define GPIO_LED_PIN_1_IOMUX                                     (IOMUX_PINCM37)
/* Defines for PIN_2: GPIOA.22 with pinCMx 47 on package pin 18 */
#define GPIO_LED_PIN_2_PORT                                              (GPIOA)
#define GPIO_LED_PIN_2_PIN                                      (DL_GPIO_PIN_22)
#define GPIO_LED_PIN_2_IOMUX                                     (IOMUX_PINCM47)
/* Defines for PIN_BEEP: GPIOB.27 with pinCMx 58 on package pin 29 */
#define GPIO_LED_PIN_BEEP_PORT                                           (GPIOB)
#define GPIO_LED_PIN_BEEP_PIN                                   (DL_GPIO_PIN_27)
#define GPIO_LED_PIN_BEEP_IOMUX                                  (IOMUX_PINCM58)
/* Defines for Key_1: GPIOA.23 with pinCMx 53 on package pin 24 */
#define GPIO_Switch_Key_1_PORT                                           (GPIOA)
#define GPIO_Switch_Key_1_PIN                                   (DL_GPIO_PIN_23)
#define GPIO_Switch_Key_1_IOMUX                                  (IOMUX_PINCM53)
/* Defines for Key_2: GPIOA.21 with pinCMx 46 on package pin 17 */
#define GPIO_Switch_Key_2_PORT                                           (GPIOA)
#define GPIO_Switch_Key_2_PIN                                   (DL_GPIO_PIN_21)
#define GPIO_Switch_Key_2_IOMUX                                  (IOMUX_PINCM46)
/* Defines for Key_3: GPIOB.18 with pinCMx 44 on package pin 15 */
#define GPIO_Switch_Key_3_PORT                                           (GPIOB)
#define GPIO_Switch_Key_3_PIN                                   (DL_GPIO_PIN_18)
#define GPIO_Switch_Key_3_IOMUX                                  (IOMUX_PINCM44)
/* Defines for Key_4: GPIOA.17 with pinCMx 39 on package pin 10 */
#define GPIO_Switch_Key_4_PORT                                           (GPIOA)
#define GPIO_Switch_Key_4_PIN                                   (DL_GPIO_PIN_17)
#define GPIO_Switch_Key_4_IOMUX                                  (IOMUX_PINCM39)


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);
void SYSCFG_DL_PWM_MOTOR_init(void);
void SYSCFG_DL_TIMER_CALC_init(void);
void SYSCFG_DL_I2C_MPU6050_init(void);
void SYSCFG_DL_I2C_OLED_init(void);
void SYSCFG_DL_UART0_init(void);
void SYSCFG_DL_UART_BT_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
