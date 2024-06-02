/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include "ds18b20.h"
#include "ssd1306.h"
#include "mcp45hv51.h"
#include "eeprom.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MAIN_MENU_IDX 		0
#define ROW_CHANGED(data)	(data & 0x10)
#define ROW_CHANGE_CLEAR  	0xcf
#define ROW_CHANGE_SET		0x10
#define ROW_SETTING_ON		0x20
#define ROW_SELECTED_MASK   0x0f

#define BUTTON_LONG_TIME		5;

#define WIPER_UPPER_STEP 		255
#define WIPER_RES				5000.0
#define WIPER_OHMxSTEP 			(WIPER_RES/WIPER_UPPER_STEP)
#define WIPER_OHM_AT_POS(pos)	((pos*WIPER_RES)/WIPER_UPPER_STEP)

#define BASE_RES 		10000
#define REF_TEMP 		(25+273.25)
#define REF_TEMP_INV	(1/REF_TEMP)

#define SETTING_POS_CHAR 12
#define SETTING_MAX_ROW 4


#define HAS_DIAG 0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
char display_row[SSD1306_WIDTH / 7 + 1];
unsigned char hasToGoInStop;
//uint8_t setTempH, setTempL;

//eeprom stored data
uint16_t __attribute__ ((section (".noinit"))) beta;
uint16_t __attribute__ ((section (".noinit"))) lowRes_value;
uint16_t __attribute__ ((section (".noinit"))) ohm_correction;
uint16_t __attribute__ ((section (".noinit"))) loopTime;
uint16_t __attribute__ ((section (".noinit"))) tempSet;
uint16_t __attribute__ ((section (".noinit"))) thresh_temp;
uint16_t __attribute__ ((section (".noinit"))) deltaT;
uint16_t __attribute__ ((section (".noinit"))) deltaR;

uint16_t loopCycles;

//         T * T0      Rt
// Beta = -------- ln ----
//         T0 - T      R0

//                           1     1
// Rt = R0 * exp ( Beta * ( --- - --- )
//                           T     T0

//             Beta
// T = ---------------------
//          Rt     Beta
//      ln ---- + ------
//          R0*     T0

//uint16_t beta;  // T1 = 25 + 273.15

// menu related
typedef void (*menuMethodPtr)(void);
typedef struct menuItem {
	uint8_t ref;
	uint8_t prev;
	uint8_t next;
	char *title;
	menuMethodPtr fptr;
	uint8_t tout;
} menuItem_t;

#if defined HAS_DIAG && HAS_DIAG == 1
menuItem_t menu[] = { { MAIN_MENU_IDX, MAIN_MENU_IDX, 1, "Main", mainmenu, 30 }, { 1, MAIN_MENU_IDX, 2, "Diagnostica", diagnostic, 30 }, { 2, 1, 3,
		"T. Settings", settingsTemp, 30 }, { 3, 2, 3, "Gen Settings", settingsGlob, 30 } };
#else
menuItem_t menu[] = { { MAIN_MENU_IDX, MAIN_MENU_IDX, 1, "Main", mainmenu, 30 }, { 1, MAIN_MENU_IDX, 2,
		"T. Settings", settingsTemp, 30 }, { 2, 1, 2, "Gen Settings", settingsGlob, 30 } };
#endif


uint8_t buttonTime = 0;
uint8_t currentMenu = 0;
uint8_t display2Update = 1;
uint8_t selectedRow = 1;
uint8_t mode;
uint8_t displayTime2Wait = 30;
// button related
// must be thread safe
uint16_t buttons_state = 0;
//uint8_t settings_mode = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	eepromInit();
	loadDataFromEEprom();
	// for Temperature Acquisition
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim2);
	//HAL_TIMEx_OCN_Start_IT(&htim1, TIM_CHANNEL_1);

	// take return value ...
	mcp45hv51_Init();

	SSD1306_Init();
	SSD1306_GotoXY(0, 0);
	SSD1306_Puts("      TRUMA", &Font_7x10, 1);
	SSD1306_GotoXY(0, 12);
	SSD1306_Puts("  Emulatore NTC", &Font_7x10, 1);
	SSD1306_GotoXY(0, 24);
//	sprintf(display_row, "WIPER POS: %u", readWiper());

	SSD1306_Puts(display_row, &Font_7x10, 1);
	SSD1306_UpdateScreen();
	HAL_Delay(1000);
	display2Update = 1;
	currentMenu = 0;
	hasToGoInStop = 0;
	uint8_t oldGoInStop = hasToGoInStop;
	while (1) {
		if(oldGoInStop == 1 && oldGoInStop != hasToGoInStop) {
//			//waked-up by external interrupt restore
			SSD1306_ON();
			SSD1306_Init();
			oldGoInStop = hasToGoInStop;
		}
		//check if timeout reached
		if(displayTime2Wait==0) {
			//discard every parameter changes
			selectedRow &= ~ROW_SETTING_ON;
			selectedRow |= ROW_CHANGE_SET;
			loadDataFromEEprom();
			//if already on main menu then shutdown the display and enable the hasToGoInStop
			if(currentMenu == MAIN_MENU_IDX) {
				hasToGoInStop = 1;
				oldGoInStop = hasToGoInStop;
				SSD1306_OFF();
			}
			else {
				//return to the main menu
				currentMenu = MAIN_MENU_IDX;
				//reload the timer for the main menu
				displayTime2Wait= menu[currentMenu].tout;
				display2Update = 1;
			}
		}
		if (display2Update != 0) {
			updateMenu();
		}

		if (hasToGoInStop == 0) {
			menu[currentMenu].fptr();
			SSD1306_UpdateScreen();
		}
		settingOperation();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//HAL_Delay(200); //system wait

		if (hasToGoInStop == 1) {
			//set a mock time
			RTC_TimeTypeDef time = { 10U, 11U, 10U };
			HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BIN);

			// set alarm time for 20 RTC cycle about 20 sec
			RTC_AlarmTypeDef alarm = { { 10U, 11U, 30U }, RTC_ALARM_A };
			HAL_RTC_SetAlarm_IT(&hrtc, &alarm, RTC_FORMAT_BIN);

			/*Suspend Tick increment to prevent wakeup by Systick interrupt.
			 Otherwise the Systick interrupt will wake up the device within 1ms (HAL time base)*/
			HAL_SuspendTick();
			//stop timer 1 to avoid interrupt
			HAL_TIM_Base_Stop_IT(&htim1);
			//stop timer 2 to avoid interrupt
			HAL_TIM_Base_Stop_IT(&htim2);
			//go in stop mode. it we waked up from alarm every 20s
			__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
			//Go in stop mode
			//HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);
			HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
			//waked up from RTC or by pressing a button
			//restore other form of interrupt
			HAL_ResumeTick();
			//restart timers
			HAL_TIM_Base_Start_IT(&htim1);
			HAL_TIM_Base_Start_IT(&htim2);

		} // end if hasToCoInStop

	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = HundredMsTimer1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 32000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 249;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DL_RED_Pin|DL_YELLOW_Pin|DL_GREEN_Pin|DL_BLUE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Wlat_Pin|LowRange_Pin|LCD_Shdn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Shdn_GPIO_Port, Shdn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : DL_RED_Pin DL_YELLOW_Pin DL_GREEN_Pin DL_BLUE_Pin */
  GPIO_InitStruct.Pin = DL_RED_Pin|DL_YELLOW_Pin|DL_GREEN_Pin|DL_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Wlat_Pin Shdn_Pin LowRange_Pin LCD_Shdn_Pin */
  GPIO_InitStruct.Pin = Wlat_Pin|Shdn_Pin|LowRange_Pin|LCD_Shdn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DS1820_IO_Pin */
  GPIO_InitStruct.Pin = DS1820_IO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DS1820_IO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SW_SU_Pin SW_SX_Pin SW_GIU_Pin */
  GPIO_InitStruct.Pin = SW_SU_Pin|SW_SX_Pin|SW_GIU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SW_DX_Pin SW_ENTER_Pin */
  GPIO_InitStruct.Pin = SW_DX_Pin|SW_ENTER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


uint8_t getWiperRFromTemp(uint8_t taH, uint8_t taL, uint16_t *resT, uint16_t *res) {
	uint8_t wiper = 0xff;
	float resistance = 10000;
	float currentTemp = 273.25;

	int16_t temp = evalTempEquival(taH, taL);
	// check if T is in the linearization zone
	int16_t dT = temp - thresh_temp;
	if (dT > (-deltaT) && dT < deltaT) {
		// in this case we apply a new and specific rule base on hysteresisR/hysteresisT
		currentTemp += (thresh_temp / 10.0);
		float resistanceBase = BASE_RES * exp(-beta * (REF_TEMP_INV - 1 / currentTemp));
		// now apply a linear constrain based on provide data
		int16_t dR = -(dT * deltaR) / deltaT; // LPP check the sign this behavior must be like an NTC
		resistance = resistanceBase + dR;
	} else {
		currentTemp += (temp / 10.0);
		resistance = BASE_RES * exp(-beta * (REF_TEMP_INV - 1 / currentTemp));
	}
	*resT=(uint16_t)resistance;
	resistance -= ohm_correction;

	if (resistance > WIPER_RES) {
		setLowRange(1);
		resistance -= lowRes_value;
	} else {
		setLowRange(0);
	}
	*res=(uint16_t)resistance;
	wiper=255;
	if (resistance <= WIPER_RES) {
		wiper = (resistance / WIPER_OHMxSTEP);
	}
	return wiper;
}


void mainmenu() {
	//{0,0,1,"Main", mainmenu, 30}
	// on first available row -> below title 7x10
	uint8_t Temp_byte; //, Temp_byte2;
	uint16_t TEMP;
	uint8_t yPos = Font_7x10.FontHeight + 1;
	//Temperature Acquisition
	DS18B20_Start();
	HAL_Delay(1);
	DS18B20_Write(0xCC);  // skip ROM
	DS18B20_Write(0x44);  // convert t
	HAL_Delay(3);
	DS18B20_Start();
	HAL_Delay(1);
	DS18B20_Write(0xCC);  // skip ROM
	DS18B20_Write(0xBE);  // Read Scratch-pad
	Temp_byte = DS18B20_Read();
	TEMP = (DS18B20_Read() << 8) | Temp_byte;
	SSD1306_GotoXY(0, yPos);
	uint8_t tempH = (TEMP / 16);
	uint8_t tempL = (TEMP * 10 / 16 - tempH * 10);
	sprintf(display_row, "Amb Temp: %2u.%1u", tempH, tempL);
	//SSD1306_Puts(display_row, &Font_11x18, 1);
	SSD1306_Puts(display_row, &Font_7x10, 1);
	// on second row -> below temp 11x18
	//yPos += Font_11x18.FontHeight+1;
	yPos += Font_7x10.FontHeight + 1;
	SSD1306_GotoXY(0, yPos);
	uint8_t setTempH = (tempSet / 10);
	uint8_t setTempL = tempSet - (setTempH * 10);

	sprintf(display_row, "Set Temp: %2u.%1u", setTempH, setTempL);
	SSD1306_Puts(display_row, &Font_7x10, 1);

	//uint8_t newWiper = getWiperFromTemp(tempH, tempL);
	uint16_t resTArg, res;
	uint8_t newWiper = getWiperRFromTemp(tempH, tempL, &resTArg, &res);
	writeWiper(newWiper);

	///// TO REMOVE ////////////////////////
	yPos += Font_7x10.FontHeight + 1;
	SSD1306_GotoXY(0, yPos);
	sprintf(display_row, "Rt=%5u;R:%5u", resTArg, res);
	SSD1306_Puts(display_row, &Font_7x10, 1);

	uint8_t wiper = readWiper();
	yPos += Font_7x10.FontHeight + 1;
	SSD1306_GotoXY(0, yPos);
//	sprintf(display_row, "Wiper: %3u", wiper);
	sprintf(display_row, "Wiper: %3u", wiper);
	SSD1306_Puts(display_row, &Font_7x10, 1);
	////////////////////////////////////////

	SSD1306_UpdateScreen(); // added also here to avoid delay

//	loopCycles = (loopTime * 5) - 1; //hyp small loop are 200ms
//	while (loopCycles > 0) {
//		HAL_Delay(200);
//		loopCycles--;
//	}

}

void settingOperation() {
	uint8_t Temp_byte; //, Temp_byte2;
	uint16_t TEMP;
	//Temperature Acquisition
	DS18B20_Start();
	HAL_Delay(1);
	DS18B20_Write(0xCC);  // skip ROM
	DS18B20_Write(0x44);  // convert t
	HAL_Delay(3);
	DS18B20_Start();
	HAL_Delay(1);
	DS18B20_Write(0xCC);  // skip ROM
	DS18B20_Write(0xBE);  // Read Scratch-pad
	Temp_byte = DS18B20_Read();
	TEMP = (DS18B20_Read() << 8) | Temp_byte;
	uint8_t tempH = (TEMP / 16);
	uint8_t tempL = (TEMP * 10 / 16 - tempH * 10);

	uint16_t r1, r2;
	uint8_t newWiper = getWiperRFromTemp(tempH, tempL, &r1, &r2);
	writeWiper(newWiper);

	loopCycles = (loopTime * 5) - 1; //hyp small loop are 200ms
	while (loopCycles > 0) {
		HAL_Delay(200);
		loopCycles--;
	}

}

void diagnostic() {
#if defined HAS_DIAG && HAS_DIAG == 1
	//{1,0,2,"Diagnostica", diagnostic, 30},
	// on first available row -> below title 7x10
	uint8_t yPos = Font_7x10.FontHeight + 1;
	uint8_t wiper = readWiper();

	int ntcValue = evalResistance(wiper);
	uint8_t equTempH;
	uint8_t equTempL;
	evalTemp(ntcValue, &equTempH, &equTempL);

	SSD1306_GotoXY(0, yPos);
	sprintf(display_row, "wiper set: %3u", wiper);
	SSD1306_Puts(display_row, &Font_7x10, 1);
	yPos += Font_7x10.FontHeight + 1;
	SSD1306_GotoXY(0, yPos);
	sprintf(display_row, "r %5d, t %2u.%1u", ntcValue, equTempH, equTempL);
	SSD1306_Puts(display_row, &Font_7x10, 1);

	uint8_t Temp_byte; //, Temp_byte2;
	uint16_t TEMP;

	//Temperature Acquisition
	DS18B20_Start();
	HAL_Delay(1);
	DS18B20_Write(0xCC);  // skip ROM
	DS18B20_Write(0x44);  // convert t
	HAL_Delay(3);
	DS18B20_Start();
	HAL_Delay(1);
	DS18B20_Write(0xCC);  // skip ROM
	DS18B20_Write(0xBE);  // Read Scratch-pad
	Temp_byte = DS18B20_Read();
	TEMP = (DS18B20_Read() << 8) | Temp_byte;
	uint8_t tempH = (TEMP / 16);
	uint8_t tempL = (TEMP * 10 / 16 - tempH * 10);

	yPos += Font_7x10.FontHeight + 1;
	SSD1306_GotoXY(0, yPos);
	wiper = getWiperFromTemp(tempH, tempL);
	ntcValue = evalResistance(wiper);
	sprintf(display_row, "Tamb: %2u.%1u", tempH, tempL);
	SSD1306_Puts(display_row, &Font_7x10, 1);
	yPos += Font_7x10.FontHeight + 1;
	SSD1306_GotoXY(0, yPos);
	sprintf(display_row, "wip:%3u res:%5u", wiper, ntcValue);
	SSD1306_Puts(display_row, &Font_7x10, 1);
#endif
}

void settingsGlob() {
	//{3,2,3,"Impostazioni", settings, 30}
	if ((selectedRow & ROW_SETTING_ON) != 0) {
		uint8_t xPos = SETTING_POS_CHAR * Font_7x10.FontWidth;
		uint8_t yPos = (selectedRow & ROW_SELECTED_MASK) * (Font_7x10.FontHeight + 2) - 1;
		SSD1306_DrawFilledRectangle(0, yPos, xPos, Font_7x10.FontHeight, SSD1306_COLOR_WHITE);
		SSD1306_DrawFilledRectangle(xPos, yPos, SSD1306_WIDTH, Font_7x10.FontHeight, SSD1306_COLOR_BLACK);
		SSD1306_GotoXY(0, yPos);
		if ((buttons_state & SU_BTN) != 0) {
			//increment value
			switch ((selectedRow & ROW_SELECTED_MASK)) {
			case 1:
				lowRes_value += 10;
				if (lowRes_value > 6000)
					lowRes_value = 6000;
				break;
			case 2:
				ohm_correction += 10;
				if (ohm_correction > 10000)
					ohm_correction = 10000;
				break;
			case 3:
				beta += 100;
				if (beta > 10000)
					beta = 10000;
				break;
			case 4:
				loopTime += 1;
				if (beta > 180)
					beta = 180;
				break;
			default:
				break;
			}
		}
		//enter in setting mode
		if ((buttons_state & GIU_BTN) != 0) {
			//decrement value
			switch ((selectedRow & ROW_SELECTED_MASK)) {
			case 1:
				lowRes_value -= 10;
				if (lowRes_value < 4000)
					lowRes_value = 4000;
				break;
			case 2:
				ohm_correction -= 10;
				if (ohm_correction < 100)
					ohm_correction = 100;

				break;
			case 3:
				beta -= 100;
				if (beta < 3000)
					beta = 3000;
				break;
			case 4:
				loopTime -= 1;
				if (beta < 1)
					beta = 1;
				break;
			default:
				break;
			}
		}

		switch ((selectedRow & ROW_SELECTED_MASK)) {
		case 1:
			sprintf(display_row, "Low Resis. :%5u", lowRes_value);
			break;
		case 2:
			sprintf(display_row, "Resistance :%5u", ohm_correction);
			break;
		case 3:
			sprintf(display_row, "Beta       :%5u", beta);
			break;
		case 4:
			sprintf(display_row, "Loop Time  :%5u", loopTime);
			break;

		default:
			break;
		}
		SSD1306_Puts_r(display_row, &Font_7x10, SETTING_POS_CHAR, SSD1306_COLOR_WHITE);
	}

	if (ROW_CHANGED(selectedRow) == 0)
		return;
	selectedRow &= ROW_CHANGE_CLEAR;
	uint8_t yPos = Font_7x10.FontHeight + 2;
	uint8_t color = SSD1306_COLOR_WHITE;
	uint8_t row = 1;
	SSD1306_DrawFilledRectangle(0, yPos - 1, SSD1306_WIDTH, SSD1306_HEIGHT - yPos, SSD1306_COLOR_BLACK);

	if (selectedRow == row) {
		color = SSD1306_COLOR_BLACK;
		SSD1306_DrawFilledRectangle(0, yPos - 1, SSD1306_WIDTH, Font_7x10.FontHeight, SSD1306_COLOR_WHITE);
	}
	SSD1306_GotoXY(0, yPos);
	sprintf(display_row, "Low Resis. :%5u", lowRes_value);
	SSD1306_Puts(display_row, &Font_7x10, color);

	row++;
	color = SSD1306_COLOR_WHITE;
	yPos += Font_7x10.FontHeight + 2;
	if (selectedRow == row) {
		color = SSD1306_COLOR_BLACK;
		SSD1306_DrawFilledRectangle(0, yPos - 1, SSD1306_WIDTH, Font_7x10.FontHeight, SSD1306_COLOR_WHITE);
	}
	SSD1306_GotoXY(0, yPos);
	sprintf(display_row, "Resistenza :%5u", ohm_correction);
	SSD1306_Puts(display_row, &Font_7x10, color);

	row++;
	color = SSD1306_COLOR_WHITE;
	yPos += Font_7x10.FontHeight + 2;
	if (selectedRow == row) {
		color = SSD1306_COLOR_BLACK;
		SSD1306_DrawFilledRectangle(0, yPos - 1, SSD1306_WIDTH, Font_7x10.FontHeight, SSD1306_COLOR_WHITE);
	}
	SSD1306_GotoXY(0, yPos);
	sprintf(display_row, "Beta       :%5u", beta);
	SSD1306_Puts(display_row, &Font_7x10, color);

	row++;
	color = SSD1306_COLOR_WHITE;
	yPos += Font_7x10.FontHeight + 2;
	if (selectedRow == row) {
		color = SSD1306_COLOR_BLACK;
		SSD1306_DrawFilledRectangle(0, yPos - 1, SSD1306_WIDTH, Font_7x10.FontHeight, SSD1306_COLOR_WHITE);
	}
	SSD1306_GotoXY(0, yPos);
	sprintf(display_row, "Loop Time  :%5u", loopTime);
	SSD1306_Puts(display_row, &Font_7x10, color);
}

void settingsTemp() {
	//{2,1,3,"Impostazioni", settings, 30}

	if ((selectedRow & ROW_SETTING_ON) != 0) {
		uint8_t xPos = SETTING_POS_CHAR * Font_7x10.FontWidth;
		uint8_t yPos = (selectedRow & ROW_SELECTED_MASK) * (Font_7x10.FontHeight + 2) - 1;
		SSD1306_DrawFilledRectangle(0, yPos, xPos, Font_7x10.FontHeight, SSD1306_COLOR_WHITE);
		SSD1306_DrawFilledRectangle(xPos, yPos, SSD1306_WIDTH, Font_7x10.FontHeight, SSD1306_COLOR_BLACK);
		SSD1306_GotoXY(0, yPos);
		if ((buttons_state & SU_BTN) != 0) {
			//increment value
			switch ((selectedRow & ROW_SELECTED_MASK)) {
			case 1:
				tempSet++;
				if (tempSet > 300)
					tempSet = 300;

				break;
			case 2:
				thresh_temp++;
				if (thresh_temp > 300)
					thresh_temp = 300;
				break;
			case 3:
				deltaT++;
				if (deltaT > 500)
					deltaT = 500;
				break;
			case 4:
				deltaR += 10;
				if (deltaR > 1000)
					deltaR = 1000;
				break;
			default:
				break;
			}
		}
		//enter in setting mode
		if ((buttons_state & GIU_BTN) != 0) {
			//decrement value
			switch ((selectedRow & ROW_SELECTED_MASK)) {
			case 1:
				tempSet--;
				if (tempSet < 100)
					tempSet = 100;
				break;
			case 2:
				thresh_temp--;
				if (thresh_temp < 100)
					thresh_temp = 100;
				break;
			case 3:
				deltaT--;
				if (deltaT < 5)
					deltaT = 5;
				break;
			case 4:
				deltaR -= 10;
				if (deltaR < 20) // minimal step for potentiometer
					deltaR = 20;
				break;
			default:
				break;
			}
		}
		uint8_t thrTempH = (thresh_temp / 10);
		uint8_t thrTempL = thresh_temp - (thrTempH * 10);
		uint8_t setTempH = (tempSet / 10);
		uint8_t setTempL = tempSet - (setTempH * 10);
		switch ((selectedRow & ROW_SELECTED_MASK)) {
		case 1:
			sprintf(display_row, "T setpoint :%3u.%1u", setTempH, setTempL);
			break;
		case 2:
			sprintf(display_row, "TrumaThr T :%3u.%1u", thrTempH, thrTempL);
			break;
		case 3:
			sprintf(display_row, "Delta Temp :%5u", deltaT);
			break;
		case 4:
			sprintf(display_row, "Delta Res. :%5u", deltaR);
			break;
		default:
			break;
		}

		SSD1306_Puts_r(display_row, &Font_7x10, SETTING_POS_CHAR, SSD1306_COLOR_WHITE);
		thresh_temp = (thrTempH * 10) + thrTempL;
	}

	if (ROW_CHANGED(selectedRow) == 0)
		return;
	selectedRow &= ROW_CHANGE_CLEAR;
	uint8_t yPos = Font_7x10.FontHeight + 2;
	uint8_t color = SSD1306_COLOR_WHITE;
	uint8_t row = 1;
	uint8_t thrTempH = (thresh_temp / 10);
	uint8_t thrTempL = thresh_temp - (thrTempH * 10);
	uint8_t setTempH = (tempSet / 10);
	uint8_t setTempL = tempSet - (setTempH * 10);
	;

	SSD1306_DrawFilledRectangle(0, yPos - 1, SSD1306_WIDTH, SSD1306_HEIGHT - yPos, SSD1306_COLOR_BLACK);
	if (selectedRow == row) {
		color = SSD1306_COLOR_BLACK;
		SSD1306_DrawFilledRectangle(0, yPos - 1, SSD1306_WIDTH, Font_7x10.FontHeight, SSD1306_COLOR_WHITE);
	}
	SSD1306_GotoXY(0, yPos);
	sprintf(display_row, "T setpoint :%3u.%1u", setTempH, setTempL);
	SSD1306_Puts(display_row, &Font_7x10, color);

	row++;
	color = SSD1306_COLOR_WHITE;
	yPos += Font_7x10.FontHeight + 2;
	if (selectedRow == row) {
		color = SSD1306_COLOR_BLACK;
		SSD1306_DrawFilledRectangle(0, yPos - 1, SSD1306_WIDTH, Font_7x10.FontHeight, SSD1306_COLOR_WHITE);
	}
	SSD1306_GotoXY(0, yPos);
	sprintf(display_row, "TrumaThr T :%3u.%1u", thrTempH, thrTempL);
	SSD1306_Puts(display_row, &Font_7x10, color);

	row++;
	color = SSD1306_COLOR_WHITE;
	yPos += Font_7x10.FontHeight + 2;
	if (selectedRow == row) {
		color = SSD1306_COLOR_BLACK;
		SSD1306_DrawFilledRectangle(0, yPos - 1, SSD1306_WIDTH, Font_7x10.FontHeight, SSD1306_COLOR_WHITE);
	}
	SSD1306_GotoXY(0, yPos);
	sprintf(display_row, "Delta Temp :%5u", deltaT);
	SSD1306_Puts(display_row, &Font_7x10, color);

	row++;
	color = SSD1306_COLOR_WHITE;
	yPos += Font_7x10.FontHeight + 2;
	if (selectedRow == row) {
		color = SSD1306_COLOR_BLACK;
		SSD1306_DrawFilledRectangle(0, yPos - 1, SSD1306_WIDTH, Font_7x10.FontHeight, SSD1306_COLOR_WHITE);
	}
	SSD1306_GotoXY(0, yPos);
	sprintf(display_row, "Delta Res. :%5u", deltaR);
	SSD1306_Puts(display_row, &Font_7x10, color);

}

void handleMenu(uint8_t sxPress) {
	if (sxPress) {
		// go back if possible
		if (menu[currentMenu].prev != currentMenu) {
			currentMenu = menu[currentMenu].prev;
			display2Update = 1; // for faster irq
			selectedRow = 1 | ROW_CHANGE_SET;
		}
	} else {
		// go next if possible
		if (menu[currentMenu].next != currentMenu) {
			currentMenu = menu[currentMenu].next;
			display2Update = 1; // for faster irq
			selectedRow = 1 | ROW_CHANGE_SET;
		}
	}
	if (menu[currentMenu].fptr != mainmenu) {
		loopCycles = 1;
	}
}

void updateMenu() {
	uint8_t nchar = strlen(menu[currentMenu].title);
	SSD1306_Clear();
	uint8_t xPos = (SSD1306_WIDTH - nchar * Font_7x10.FontWidth) / 2;
	SSD1306_GotoXY(xPos, 0);
	SSD1306_Puts(menu[currentMenu].title, &Font_7x10, 1);
	if (menu[currentMenu].prev != currentMenu) {
		SSD1306_GotoXY(0, 0);
		SSD1306_Putc('<', &Font_7x10, 1);
	}
	if (menu[currentMenu].next != currentMenu) {
		SSD1306_GotoXY(SSD1306_WIDTH - Font_7x10.FontWidth - 1, 0);
		SSD1306_Putc('>', &Font_7x10, 1);
	}
	display2Update = 0;
	displayTime2Wait=menu[currentMenu].tout;
}

void loadDataFromEEprom() {

	lowRes_value = 5000;
	ohm_correction = 7500;
	beta = 3900;
	loopTime = 1; // tempo in secondi
	tempSet = 210;

	thresh_temp = 210;
	deltaT = 5; // in decimi di grado
	deltaR = 240; // estratta da variazione curva con Delta di 1 grado a 23gradi

	eepromLoadParam(RC_KEY1, &beta);
	eepromLoadParam(RC_KEY2, &lowRes_value);
	eepromLoadParam(RC_KEY3, &ohm_correction);
	eepromLoadParam(RC_KEY4, &loopTime);

	eepromLoadParam(RC_KEY5, &tempSet);
	eepromLoadParam(RC_KEY6, &thresh_temp);
	eepromLoadParam(RC_KEY7, &deltaT);
	eepromLoadParam(RC_KEY8, &deltaR);

}

void writeDataOnEEprom() {
#ifdef DEBUG
	return;
#endif

	eepromSaveParam(RC_KEY1, &beta, sizeof(beta));
	eepromSaveParam(RC_KEY2, &lowRes_value, sizeof(lowRes_value));
	eepromSaveParam(RC_KEY3, &ohm_correction, sizeof(ohm_correction));
	eepromSaveParam(RC_KEY4, &loopTime, sizeof(loopTime));
	eepromSaveParam(RC_KEY5, &tempSet, sizeof(tempSet));
	eepromSaveParam(RC_KEY6, &thresh_temp, sizeof(thresh_temp));
	eepromSaveParam(RC_KEY7, &deltaT, sizeof(deltaT));
	eepromSaveParam(RC_KEY8, &deltaR, sizeof(deltaR));
}

/**
 * translate the ambient temperature the same amount as the distance between the Truma setpoint and the wanted setpoint
 * the dT = Ta - Ts must be equal to dT1 = Te - Tt
 * where Ta = ambient temperature
 * where Ts = setpoint temperature
 * where Te = equivalent temperature
 * where Tt = truma temperature
 * Ta - Ts = Te - Tt  --> Te = Ta - Ts + Tt
 */
int16_t evalTempEquival(uint8_t taH, uint8_t taL) {
	int16_t tempE = (taH * 10) - tempSet + thresh_temp + taL;
	return tempE;
}
/**
 * evaluate the exposed resistance based on the wiper position
 */
int evalResistance(uint8_t wiper) {

	int res = 0;
#if defined HAS_DIAG && HAS_DIAG == 1
	if (HAL_GPIO_ReadPin(LowRange_GPIO_Port, LowRange_Pin) == GPIO_PIN_RESET) {
		res = lowRes_value;
	}
	res += ohm_correction;
	res += WIPER_OHM_AT_POS(wiper);
#endif
	return res;
}

/**
 * evaluate the equivalent temperature based on the NTC resistance
 *             Beta
 * T = ---------------------
 *         Rt     Beta
 *     ln ---- + ------
 *         R0*     T0
 */
void evalTemp(int ntcValue, uint8_t *equTempH, uint8_t *equTempL) {
#if defined HAS_DIAG && HAS_DIAG == 1
	//LPP doens't work properly the result ins't correct
	float denom = log((float) ntcValue / (float) BASE_RES);
	denom += (beta / REF_TEMP);
	float temp = (beta / denom) - 273.25;
	*equTempH = floor(temp);
	*equTempL = (temp - (float) (*equTempH)) * 10;
#endif
} // end evalTemp

/**
 * get the wiper setting from the provided temperature
 *                           1     1
 * Rt = R0 * exp ( Beta * ( --- - --- )
 *                           T     T0
 *
 */
uint8_t getWiperFromTemp(uint8_t taH, uint8_t taL) {

	uint8_t wiper = 0xff;
#if defined HAS_DIAG && HAS_DIAG == 1
	float resistance = 10000;
	float currentTemp = 273.25;

	int16_t temp = evalTempEquival(taH, taL);
	// check if T is in the linearization zone
	int16_t dT = temp - thresh_temp;
	if (dT > (-deltaT) && dT < deltaT) {
		// in this case we apply a new and specific rule base on hysteresisR/hysteresisT
		currentTemp += (thresh_temp / 10.0);
		float resistanceBase = BASE_RES * exp(-beta * (REF_TEMP_INV - 1 / currentTemp));
		// now apply a linear constrain based on provide data
		int16_t dR = -(dT * deltaR) / deltaT; // LPP check the sign this behavior must be like an NTC
		resistance = resistanceBase + dR;
	} // end if Temperature distance is inside the required bounds
	else {
		currentTemp += (temp / 10.0);
		resistance = BASE_RES * exp(-beta * (REF_TEMP_INV - 1 / currentTemp));
	} //end else the temperature is far from the thresholds

	resistance -= ohm_correction;

	if (resistance > WIPER_RES) {
		setLowRange(1); //insert the low value resistor
		resistance -= lowRes_value;
	} // end if the resistance value is greater than the maximum pot value
	else {
		setLowRange(0); // shorts the low value resistor
	}

	wiper=255;
	if (resistance <= WIPER_RES) {
		wiper = (resistance / WIPER_OHMxSTEP);
	}
#endif
	return wiper;
}

/**
 * set the pin level accordingly to the required LowLevel
 * an high level on the LowRange_Pin disable the switch so the resistance is higher
 * a low level on the LowRange_Pin close the switch so the resistance is lower
 */
void setLowRange(uint8_t isLowRange) {
	if (isLowRange == 0) {
		HAL_GPIO_WritePin(LowRange_GPIO_Port, LowRange_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DL_BLUE_GPIO_Port, DL_BLUE_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(LowRange_GPIO_Port, LowRange_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DL_BLUE_GPIO_Port, DL_BLUE_Pin, GPIO_PIN_RESET);
	}
}

/**
 * retrieve if the LowLevel is set or not
 */
uint8_t getLowRange() {
	uint8_t ret = 1;
	if (HAL_GPIO_ReadPin(LowRange_GPIO_Port, LowRange_Pin) == GPIO_PIN_RESET)
		ret = 0;
	return ret;
}

/**
 * command the electronic potentiometer in order to set the wiper at the required value
 */
void setWiper(uint8_t wiper) {

	writeWiper(wiper);
}

/**
 *
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	GPIO_TypeDef *port;
	uint16_t pin;
	uint8_t valid = 0;
	uint8_t btn;

	switch (GPIO_Pin) {
	case SW_DX_Pin:
		port = SW_DX_GPIO_Port;
		pin = SW_DX_Pin;
		btn = DX_BTN;
		valid = 1;
		break;
	case SW_ENTER_Pin:
		port = SW_ENTER_GPIO_Port;
		pin = SW_ENTER_Pin;
		btn = ENTER_BTN;
		valid = 1;
		break;
	case SW_GIU_Pin:
		port = SW_GIU_GPIO_Port;
		pin = SW_GIU_Pin;
		btn = GIU_BTN;
		valid = 1;
		break;
	case SW_SX_Pin:
		port = SW_SX_GPIO_Port;
		pin = SW_SX_Pin;
		btn = SX_BTN;
		valid = 1;
		break;
	case SW_SU_Pin:
		port = SW_SU_GPIO_Port;
		pin = SW_SU_Pin;
		btn = SU_BTN;
		valid = 1;
		break;
	default:
		break;
	}

	if (valid == 1) {
		//check if hasToGoInStop is active in such case re-init the display and clear the hasToGoInStop
		if(hasToGoInStop == 1) {
			hasToGoInStop = 0;
			display2Update = 1;
		}

		displayTime2Wait=menu[currentMenu].tout; // reset the timer on every event
		if (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET) {
			buttons_state = btn;
			buttonTime = BUTTON_LONG_TIME;
		} else {
			//buttons_state &= ~(BTN_LONG_MASK);
			buttons_state = 0;
			buttonTime = 0; // reset time
			if (pin == SW_SX_Pin)
				handleMenu(1);
			if (pin == SW_DX_Pin)
				handleMenu(0);
			if (pin == SW_GIU_Pin) {
				//if not in setting then change row otherwise is handled into setting menu
				if ((selectedRow & ROW_SETTING_ON) == 0) {
					selectedRow++;
					if (selectedRow > SETTING_MAX_ROW)
						selectedRow = SETTING_MAX_ROW;
					selectedRow |= ROW_CHANGE_SET;
				}
			}
			if (pin == SW_SU_Pin) {
				//if not in setting then change row otherwise is handled into setting menu
				if ((selectedRow & ROW_SETTING_ON) == 0) {
					selectedRow--;
					if (selectedRow < 1)
						selectedRow = 1;
					selectedRow |= ROW_CHANGE_SET;
				}
			}
			if (pin == SW_ENTER_Pin) {
				//toggle the SETTING ON
				if ((selectedRow & ROW_SETTING_ON) != 0) {
					selectedRow &= ~ROW_SETTING_ON;
					selectedRow |= ROW_CHANGE_SET;
					writeDataOnEEprom();
				} else {
					selectedRow |= ROW_SETTING_ON;
				}
			}
		}
	}

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
