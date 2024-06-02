/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void handleMenu(uint8_t sxPress);
void updateMenu();
void mainmenu();
void diagnostic();
void settingsTemp();
void settingsGlob();
void loadDataFromEEprom();
void writeDataOnEEprom();
int evalResistance(uint8_t wiper);
void evalTemp(int ntcValue, uint8_t *equTempH, uint8_t *equTempL);
int16_t evalTempEquival(uint8_t taH, uint8_t taL);
uint8_t getWiperFromTemp(uint8_t taH, uint8_t taL);

void setLowRange(uint8_t isLowRange);
uint8_t getLowRange();
void settingOperation();
void setWiper(uint8_t wiper);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define mcp_addr 0x3c
#define HundredMsTimer1 99
#define DL_RED_Pin GPIO_PIN_4
#define DL_RED_GPIO_Port GPIOA
#define DL_YELLOW_Pin GPIO_PIN_5
#define DL_YELLOW_GPIO_Port GPIOA
#define DL_GREEN_Pin GPIO_PIN_6
#define DL_GREEN_GPIO_Port GPIOA
#define DL_BLUE_Pin GPIO_PIN_7
#define DL_BLUE_GPIO_Port GPIOA
#define Wlat_Pin GPIO_PIN_0
#define Wlat_GPIO_Port GPIOB
#define Shdn_Pin GPIO_PIN_1
#define Shdn_GPIO_Port GPIOB
#define DS1820_IO_Pin GPIO_PIN_2
#define DS1820_IO_GPIO_Port GPIOB
#define LowRange_Pin GPIO_PIN_12
#define LowRange_GPIO_Port GPIOB
#define SW_SU_Pin GPIO_PIN_13
#define SW_SU_GPIO_Port GPIOB
#define SW_SU_EXTI_IRQn EXTI15_10_IRQn
#define SW_SX_Pin GPIO_PIN_14
#define SW_SX_GPIO_Port GPIOB
#define SW_SX_EXTI_IRQn EXTI15_10_IRQn
#define SW_GIU_Pin GPIO_PIN_15
#define SW_GIU_GPIO_Port GPIOB
#define SW_GIU_EXTI_IRQn EXTI15_10_IRQn
#define SW_DX_Pin GPIO_PIN_8
#define SW_DX_GPIO_Port GPIOA
#define SW_DX_EXTI_IRQn EXTI9_5_IRQn
#define SW_ENTER_Pin GPIO_PIN_9
#define SW_ENTER_GPIO_Port GPIOA
#define SW_ENTER_EXTI_IRQn EXTI9_5_IRQn
#define LCD_Shdn_Pin GPIO_PIN_3
#define LCD_Shdn_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
//eeprom variable type
#define RC_KEY1		0
#define RC_KEY2		1
#define RC_KEY3		2
#define RC_KEY4		3
#define RC_KEY5		4
#define RC_KEY6		5
#define RC_KEY7		6
#define RC_KEY8		7

#define ENTER_BTN 0x0001
#define DX_BTN    0x0002
#define GIU_BTN   0x0004
#define SX_BTN    0x0008
#define SU_BTN    0x0010
#define BTN_MASK (0x01<< 5)-1 //n buttons
//#define BTN_RELEASED_MASK 0x0020
#define BTN_LONG_MASK 0x0020


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
