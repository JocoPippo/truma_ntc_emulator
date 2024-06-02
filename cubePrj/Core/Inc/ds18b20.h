/*
 * ds18b20.h
 *
 *  Created on: Feb 18, 2024
 *      Author: Roberto
 */

#ifndef INC_DS18B20_H_
#define INC_DS18B20_H_

void delay (uint16_t time);
void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
uint8_t DS18B20_Start (void);
void DS18B20_Write (uint8_t data);
uint8_t DS18B20_Read (void);



#endif /* INC_DS18B20_H_ */
