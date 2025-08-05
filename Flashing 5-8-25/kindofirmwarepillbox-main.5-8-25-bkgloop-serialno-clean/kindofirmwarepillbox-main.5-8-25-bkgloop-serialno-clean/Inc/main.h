/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "stm32f0xx_hal.h"
#include "stdio.h"
#include <stdlib.h>
#include <math.h>
#include "string.h"
#include "mpu6050.h"
#include "stepper.h"
#include "rc522.h"
#include "sk6812.h"
#include "protocol.h"
#include "function.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "icm40608.h"
#include "iwdg.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

//extern SPI_HandleTypeDef hspi1;

extern TIM_HandleTypeDef htim1;



extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;


extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
//uint16_t lasttime, time1;

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void __delay_ms(int32_t k);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define STP1_EN_Pin GPIO_PIN_2
#define STP1_EN_GPIO_Port GPIOE
#define STP1_DIR_Pin GPIO_PIN_3
#define STP1_DIR_GPIO_Port GPIOE
#define STP1_FAULT_Pin GPIO_PIN_4
#define STP1_FAULT_GPIO_Port GPIOE
#define STP2_DIR_Pin GPIO_PIN_5
#define STP2_DIR_GPIO_Port GPIOE
#define STP2_EN_Pin GPIO_PIN_6
#define STP2_EN_GPIO_Port GPIOE
#define STP2_FAULT_Pin GPIO_PIN_13
#define STP2_FAULT_GPIO_Port GPIOC
#define DEBUG_TX_Pin GPIO_PIN_9
#define DEBUG_TX_GPIO_Port GPIOF
#define DEBUG_RX_Pin GPIO_PIN_10
#define DEBUG_RX_GPIO_Port GPIOF
#define AIN_SENS_Pin GPIO_PIN_0
#define AIN_SENS_GPIO_Port GPIOC
#define CE_MCU_WAKEUP_Pin GPIO_PIN_1
#define CE_MCU_WAKEUP_GPIO_Port GPIOC
#define CE_MCU_WAKEUP_EXTI_IRQn EXTI0_1_IRQn
#define VBAT_SENS_Pin GPIO_PIN_2
#define VBAT_SENS_GPIO_Port GPIOC
#define CPU_EN_Pin GPIO_PIN_3
#define CPU_EN_GPIO_Port GPIOC
#define DCIN_DET_Pin GPIO_PIN_2
#define DCIN_DET_GPIO_Port GPIOF
#define DCIN_DET_EXTI_IRQn EXTI2_3_IRQn
#define SYS_EN_Pin GPIO_PIN_3
#define SYS_EN_GPIO_Port GPIOF
#define TAMP_SENS_Pin GPIO_PIN_0
#define TAMP_SENS_GPIO_Port GPIOA
#define PNEU_ATM_Pin GPIO_PIN_1
#define PNEU_ATM_GPIO_Port GPIOA
#define BATT_CHRG_Pin GPIO_PIN_2
#define BATT_CHRG_GPIO_Port GPIOA
#define BATT_STDBY_Pin GPIO_PIN_3
#define BATT_STDBY_GPIO_Port GPIOA
#define CROT_ENCI_Pin GPIO_PIN_6
#define CROT_ENCI_GPIO_Port GPIOA
#define CROT_ENCI_EXTI_IRQn EXTI4_15_IRQn
#define PROT_ENCI_Pin GPIO_PIN_7
#define PROT_ENCI_GPIO_Port GPIOA
#define PROT_ENCI_EXTI_IRQn EXTI4_15_IRQn
#define PILLBX_DET_Pin GPIO_PIN_0
#define PILLBX_DET_GPIO_Port GPIOB
#define STP3_STEP_Pin GPIO_PIN_1
#define STP3_STEP_GPIO_Port GPIOB
#define STP3_EN_Pin GPIO_PIN_2
#define STP3_EN_GPIO_Port GPIOB
#define STP3_DIR_Pin GPIO_PIN_7
#define STP3_DIR_GPIO_Port GPIOE
#define STP3_FAULT_Pin GPIO_PIN_8
#define STP3_FAULT_GPIO_Port GPIOE
#define PNEU_PUMP_Pin GPIO_PIN_9
#define PNEU_PUMP_GPIO_Port GPIOE
#define PNEU_VALV_Pin GPIO_PIN_10
#define PNEU_VALV_GPIO_Port GPIOE
#define ARM_X_STP1_Pin GPIO_PIN_11
#define ARM_X_STP1_GPIO_Port GPIOE
#define ARM_X_STP2_Pin GPIO_PIN_12
#define ARM_X_STP2_GPIO_Port GPIOE
#define ARM_Z_STP1_Pin GPIO_PIN_13
#define ARM_Z_STP1_GPIO_Port GPIOE
#define ARM_Z_STP2_Pin GPIO_PIN_14
#define ARM_Z_STP2_GPIO_Port GPIOE
#define MPU_INT_Pin GPIO_PIN_15
#define MPU_INT_GPIO_Port GPIOE
#define AUX_SCL_Pin GPIO_PIN_10
#define AUX_SCL_GPIO_Port GPIOB
#define AUX_SDA_Pin GPIO_PIN_11
#define AUX_SDA_GPIO_Port GPIOB
#define STP5_DIR_Pin GPIO_PIN_12
#define STP5_DIR_GPIO_Port GPIOB
#define STP5_EN_Pin GPIO_PIN_13
#define STP5_EN_GPIO_Port GPIOB
#define STP5_STEP_Pin GPIO_PIN_14
#define STP5_STEP_GPIO_Port GPIOB
#define STP5_FAULT_Pin GPIO_PIN_15
#define STP5_FAULT_GPIO_Port GPIOB
#define CROT_ENCZ_Pin GPIO_PIN_8
#define CROT_ENCZ_GPIO_Port GPIOD
#define CROT_ENCZ_EXTI_IRQn EXTI4_15_IRQn
#define PROT_ENCZ_Pin GPIO_PIN_9
#define PROT_ENCZ_GPIO_Port GPIOD
#define PROT_ENCZ_EXTI_IRQn EXTI4_15_IRQn
#define FRNTDR_CLS_Pin GPIO_PIN_10
#define FRNTDR_CLS_GPIO_Port GPIOD
#define PILLDR_OPN_Pin GPIO_PIN_11
#define PILLDR_OPN_GPIO_Port GPIOD
#define PILLDR_CLS_Pin GPIO_PIN_12
#define PILLDR_CLS_GPIO_Port GPIOD
#define FRNTDR_LCK_Pin GPIO_PIN_13
#define FRNTDR_LCK_GPIO_Port GPIOD
#define FRNTDR_MAG_Pin GPIO_PIN_14
#define FRNTDR_MAG_GPIO_Port GPIOD
#define PILLBX_HALL_Pin GPIO_PIN_15
#define PILLBX_HALL_GPIO_Port GPIOD
#define FRNTDR_LED0_Pin GPIO_PIN_6
#define FRNTDR_LED0_GPIO_Port GPIOC
#define PILLBX_LLED0_Pin GPIO_PIN_7
#define PILLBX_LLED0_GPIO_Port GPIOC
#define PILLBX_RLED0_Pin GPIO_PIN_8
#define PILLBX_RLED0_GPIO_Port GPIOC
#define STP4_STEP_Pin GPIO_PIN_8
#define STP4_STEP_GPIO_Port GPIOA
#define DFU_TX_Pin GPIO_PIN_9
#define DFU_TX_GPIO_Port GPIOA
#define DFU_RX_Pin GPIO_PIN_10
#define DFU_RX_GPIO_Port GPIOA
#define STP4_EN_Pin GPIO_PIN_11
#define STP4_EN_GPIO_Port GPIOA
#define STP4_DIR_Pin GPIO_PIN_12
#define STP4_DIR_GPIO_Port GPIOA
#define STP4_FAULT_Pin GPIO_PIN_6
#define STP4_FAULT_GPIO_Port GPIOF
#define RFID_NSS_Pin GPIO_PIN_15
#define RFID_NSS_GPIO_Port GPIOA
#define AD_SYNC_Pin GPIO_PIN_0
#define AD_SYNC_GPIO_Port GPIOD
#define AD_SCK_Pin GPIO_PIN_1
#define AD_SCK_GPIO_Port GPIOD
#define AD_LDAC_Pin GPIO_PIN_2
#define AD_LDAC_GPIO_Port GPIOD
#define A33_PWR_Pin GPIO_PIN_3
#define A33_PWR_GPIO_Port GPIOD
#define AD_DIN_Pin GPIO_PIN_4
#define AD_DIN_GPIO_Port GPIOD
#define CMD_TX_Pin GPIO_PIN_5
#define CMD_TX_GPIO_Port GPIOD
#define CMD_RX_Pin GPIO_PIN_6
#define CMD_RX_GPIO_Port GPIOD
#define FRNTDR_BUTT_Pin GPIO_PIN_7
#define FRNTDR_BUTT_GPIO_Port GPIOD
#define RFID_SCK_Pin GPIO_PIN_3
#define RFID_SCK_GPIO_Port GPIOB
#define RFID_MISO_Pin GPIO_PIN_4
#define RFID_MISO_GPIO_Port GPIOB
#define RFID_MOSI_Pin GPIO_PIN_5
#define RFID_MOSI_GPIO_Port GPIOB
#define RFID_IRQ_Pin GPIO_PIN_6
#define RFID_IRQ_GPIO_Port GPIOB
#define RFID_NRST_Pin GPIO_PIN_7
#define RFID_NRST_GPIO_Port GPIOB
#define STP1_STEP_Pin GPIO_PIN_0
#define STP1_STEP_GPIO_Port GPIOE
#define STP2_STEP_Pin GPIO_PIN_1
#define STP2_STEP_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */
#define LLED 0
#define TWOLED 1
#define	RLED 1
#define LED3 2


//stepper
#define ARM_X 3
#define ARM_Z 4
#define ROT_Y 0
#define rotProt 1
#define ChuteDoor 2

//adc
#define PNEU_ADC 0
#define PILLBOX_DET_ADC 1
#define STP_Z_CURRENT 2


//led
//#define off 0
#define blink 1
#define longon 2
#define change 3

#define PILLBX_DET adc_8
#define PILLBX_DET adc_8
extern uint32_t rottime, piltime;
extern uint8_t targ_x, targ_z,checkpressure,ismove_x,ismove_y,ismove_z,returnstate,keepon;

extern uint16_t overtemp, overroll, overpitch, overhum;
extern uint8_t dacZ, dacY, dacPill, dacDoor; 
extern uint32_t value_adc[12];
void ReadNFCPN532(void);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
