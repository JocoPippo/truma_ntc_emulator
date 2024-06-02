/*
 * ds18b20.c
 *
 *  Created on: Feb 18, 2024
 *      Author: Roberto
 */

#include "main.h"
#include "ds18b20.h"
#include <stm32f1xx_hal_tim.h>

extern TIM_HandleTypeDef htim3;
#define oneUSecTimerHandle htim3


void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}


//#define DS18B20_PIN GPIO_PIN_1

uint8_t DS18B20_Start (void)
{
	uint8_t Response = 0;
	Set_Pin_Output(DS1820_IO_GPIO_Port, DS1820_IO_Pin);   // set the pin as output
	HAL_GPIO_WritePin (DS1820_IO_GPIO_Port, DS1820_IO_Pin, 0);  // pull the pin low
	delay (480);   // delay according to datasheet

	Set_Pin_Input(DS1820_IO_GPIO_Port, DS1820_IO_Pin);    // set the pin as input
	delay (80);    // delay according to datasheet

	if (!(HAL_GPIO_ReadPin (DS1820_IO_GPIO_Port, DS1820_IO_Pin))) Response = 1;    // if the pin is low i.e the presence pulse is detected
	else Response = -1;

	delay (400); // 480 us delay totally.

	return Response;
}

void DS18B20_Write (uint8_t data)
{
	Set_Pin_Output(DS1820_IO_GPIO_Port, DS1820_IO_Pin);  // set as output

	for (int i=0; i<8; i++)
	{

		if ((data & (1<<i))!=0)  // if the bit is high
		{
			// write 1

			Set_Pin_Output(DS1820_IO_GPIO_Port, DS1820_IO_Pin);  // set as output
			HAL_GPIO_WritePin (DS1820_IO_GPIO_Port, DS1820_IO_Pin, 0);  // pull the pin LOW
			delay (1);  // wait for 1 us

			Set_Pin_Input(DS1820_IO_GPIO_Port, DS1820_IO_Pin);  // set as input
			delay (50);  // wait for 60 us
		}

		else  // if the bit is low
		{
			// write 0

			Set_Pin_Output(DS1820_IO_GPIO_Port, DS1820_IO_Pin);
			HAL_GPIO_WritePin (DS1820_IO_GPIO_Port, DS1820_IO_Pin, 0);  // pull the pin LOW
			delay (50);  // wait for 60 us

			Set_Pin_Input(DS1820_IO_GPIO_Port, DS1820_IO_Pin);
		}
	}
}

uint8_t DS18B20_Read (void)
{
	uint8_t value=0;

	Set_Pin_Input(DS1820_IO_GPIO_Port, DS1820_IO_Pin);

	for (int i=0;i<8;i++)
	{
		Set_Pin_Output(DS1820_IO_GPIO_Port, DS1820_IO_Pin);   // set as output

		HAL_GPIO_WritePin (DS1820_IO_GPIO_Port, DS1820_IO_Pin, 0);  // pull the data pin LOW
		delay (1);  // wait for > 1us

		Set_Pin_Input(DS1820_IO_GPIO_Port, DS1820_IO_Pin);  // set as input
		if (HAL_GPIO_ReadPin (DS1820_IO_GPIO_Port, DS1820_IO_Pin))  // if the pin is HIGH
		{
			value |= 1<<i;  // read = 1
		}
		delay (50);  // wait for 60 us
	}
	return value;
}

void delay (uint16_t time)
{
	/* change your code here for the delay in microseconds */
	__HAL_TIM_SET_COUNTER(&oneUSecTimerHandle, 0);
	while ((__HAL_TIM_GET_COUNTER(&oneUSecTimerHandle))<time);
}
