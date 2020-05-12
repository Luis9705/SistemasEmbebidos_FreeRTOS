/*
 * LED.c
 *
 *  Created on: May 9, 2020
 *      Author: FLL1GA
 */

/// @file i2c_adc.c
/// @brief Functions to use I2C_ADC

#include "LED.h"		///<LED header
#include "gpio.h"		///<GPIO header

led_statusType sensing_led_status = LED_OFF;
led_statusType min_led_status = LED_OFF;
led_statusType max_led_status = LED_OFF;


/**
 * @brief Sets up functions needed to manage the LED.
 * @retval None
 */
void LED_setup(void){
	MX_GPIO_Init();

	sensingLED_OFF();
	maxTempLED_OFF();
	minTempLED_OFF();
	sensing_led_status = LED_OFF;
	min_led_status = LED_OFF;
	max_led_status = LED_OFF;
}

/**
 * @brief Turns on the "sensing" LED
 * @retval None
 */
void sensingLED_ON(void){
	HAL_GPIO_WritePin(LED_BOARD_GPIO_Port, LED_BOARD_Pin, GPIO_PIN_RESET);
	sensing_led_status = LED_ON;
}

/**
 * @brief Turns off the "sensing" LED
 * @retval None
 */
void sensingLED_OFF(void){
	HAL_GPIO_WritePin(LED_BOARD_GPIO_Port, LED_BOARD_Pin, GPIO_PIN_SET);
	sensing_led_status = LED_OFF;
}

/**
 * @brief Toggles the "sensing" LED
 * @retval None
 */
void sensingLED_TOGGLE(void){
	HAL_GPIO_TogglePin(LED_BOARD_GPIO_Port, LED_BOARD_Pin);
	sensing_led_status = sensing_led_status == LED_ON ? LED_OFF : LED_ON;
}
/**
 * @brief Turns on the "sensing" LED
 * @return sensing_led_status
 */
led_statusType getSensingLED_STATUS(void){
	return sensing_led_status;
}

/**
 * @brief Turns on the Max Temperature LED.
 * @retval None
 */
void maxTempLED_ON(void){
	HAL_GPIO_WritePin(LED_MAX_GPIO_Port, LED_MAX_Pin, GPIO_PIN_RESET);
	max_led_status = LED_ON;
}
/**
 * @brief Turns off the Max Temperature LED.
 * @retval None
 */
void maxTempLED_OFF(void){
	HAL_GPIO_WritePin(LED_MAX_GPIO_Port, LED_MAX_Pin, GPIO_PIN_SET);
	max_led_status = LED_OFF;

}
/**
 * @brief Toggles the Max Temperature LED.
 * @retval None
 */
void maxTempLED_TOGGLE(void){
	HAL_GPIO_TogglePin(LED_MAX_GPIO_Port, LED_MAX_Pin);
	max_led_status = max_led_status == LED_ON ? LED_OFF : LED_ON;
}

/**
 * @brief Returns the Max Temperature LED status.
 * @return max_led_status
 */
led_statusType getMaxLED_STATUS(void){
	return max_led_status;
}

/**
 * @brief Turns on the Min Temperature LED.
 * @retval None
 */
void minTempLED_ON(void){
	HAL_GPIO_WritePin(LED_MIN_GPIO_Port, LED_MIN_Pin, GPIO_PIN_RESET);
	min_led_status = LED_ON;

}
/**
 * @brief Turns off the Min Temperature LED.
 * @retval None
 */
void minTempLED_OFF(void){
	HAL_GPIO_WritePin(LED_MIN_GPIO_Port, LED_MIN_Pin, GPIO_PIN_SET);
	min_led_status = LED_OFF;

}
/**
 * @brief Toggles the Min Temperature LED.
 * @retval None
 */
void minTempLED_TOGGLE(void){
	HAL_GPIO_TogglePin(LED_MIN_GPIO_Port, LED_MIN_Pin);
	min_led_status = min_led_status == LED_ON ? LED_OFF : LED_ON;
}
/**
 * @brief Turns on the Min Temperature LED.
 * @return min_led_status
 */
led_statusType getMinLED_STATUS(void){
	return min_led_status;
}
