/*
 * LED.c
 *
 *  Created on: May 9, 2020
 *      Author: FLL1GA
 */

#include "LED.h"
#include "gpio.h"

led_statusType sensing_led_status = LED_OFF;
led_statusType min_led_status = LED_OFF;
led_statusType max_led_status = LED_OFF;


void LED_setup(void){
	MX_GPIO_Init();

	sensingLED_OFF();
	maxTempLED_OFF();
	minTempLED_OFF();
	sensing_led_status = LED_OFF;
	min_led_status = LED_OFF;
	max_led_status = LED_OFF;
}

void sensingLED_ON(void){
	HAL_GPIO_WritePin(LED_BOARD_GPIO_Port, LED_BOARD_Pin, GPIO_PIN_RESET);
	sensing_led_status = LED_ON;
}

void sensingLED_OFF(void){
	HAL_GPIO_WritePin(LED_BOARD_GPIO_Port, LED_BOARD_Pin, GPIO_PIN_SET);
	sensing_led_status = LED_OFF;
}
void sensingLED_TOGGLE(void){
	HAL_GPIO_TogglePin(LED_BOARD_GPIO_Port, LED_BOARD_Pin);
	sensing_led_status = sensing_led_status == LED_ON ? LED_OFF : LED_ON;
}
led_statusType getSensingLED_STATUS(void){
	return sensing_led_status;
}

void maxTempLED_ON(void){
	HAL_GPIO_WritePin(LED_MAX_GPIO_Port, LED_MAX_Pin, GPIO_PIN_RESET);
	max_led_status = LED_ON;
}
void maxTempLED_OFF(void){
	HAL_GPIO_WritePin(LED_MAX_GPIO_Port, LED_MAX_Pin, GPIO_PIN_SET);
	max_led_status = LED_OFF;

}
void maxTempLED_TOGGLE(void){
	HAL_GPIO_TogglePin(LED_MAX_GPIO_Port, LED_MAX_Pin);
	max_led_status = max_led_status == LED_ON ? LED_OFF : LED_ON;
}
led_statusType getMaxLED_STATUS(void){
	return max_led_status;
}

void minTempLED_ON(void){
	HAL_GPIO_WritePin(LED_MIN_GPIO_Port, LED_MIN_Pin, GPIO_PIN_RESET);
	min_led_status = LED_ON;

}
void minTempLED_OFF(void){
	HAL_GPIO_WritePin(LED_MIN_GPIO_Port, LED_MIN_Pin, GPIO_PIN_SET);
	min_led_status = LED_OFF;

}
void minTempLED_TOGGLE(void){
	HAL_GPIO_TogglePin(LED_MIN_GPIO_Port, LED_MIN_Pin);
	min_led_status = min_led_status == LED_ON ? LED_OFF : LED_ON;
}
led_statusType getMinLED_STATUS(void){
	return min_led_status;
}
