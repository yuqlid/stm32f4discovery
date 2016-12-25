/*
 * led.c
 *
 *  Created on: 2016/12/24
 *      Author: Yuki
 */

#include "led.h"
#include "gpio.h"

void led_on(Led_TypeDef led){
	if(led & LED3)	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin , GPIO_PIN_SET);

	if(led & LED4)	HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin , GPIO_PIN_SET);

	if(led & LED5)	HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin , GPIO_PIN_SET);

	if(led & LED6)	HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin , GPIO_PIN_SET);
}

void led_off(Led_TypeDef led){
	if(led & LED3)	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin , GPIO_PIN_RESET);

	if(led & LED4)	HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin , GPIO_PIN_RESET);

	if(led & LED5)	HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin , GPIO_PIN_RESET);

	if(led & LED6)	HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin , GPIO_PIN_RESET);
}

void led_toggle(Led_TypeDef led){
	if(led & LED3)	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

	if(led & LED4)	HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);

	if(led & LED5)	HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);

	if(led & LED6)	HAL_GPIO_TogglePin(LD6_GPIO_Port, LD6_Pin);
}

