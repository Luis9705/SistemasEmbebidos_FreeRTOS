/*
 * uc_adc.c
 *
 *  Created on: May 2, 2020
 *      Author: FLL1GA
 */

/// @file uc_adc.c
//  Copyright 2020 Copyright Equipo 2
#include "uc_adc.h"
#include "main.h"


ADC_HandleTypeDef * hadc;
/**
 * Sets up the ADC peripheral pin ports needed.
 */
void adc_pin_setup(void) {
	//adc pin initializing in main function
}

/**
 * Sets up the ADC peripheral.
 */
void adc_setup(void * init_struct) {

	hadc = (ADC_HandleTypeDef *)init_struct;
	//adc initializing in main function
}

/**
 * Reads the ADC conversion value.
 * @param[out] adc_read_value.
 */
uint16_t adc_read(void) {

	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
	return HAL_ADC_GetValue(&hadc);

}

/**
 * Reads the ADC value and scales it up to volts.
 * @param[out] voltage.
 */
float adc_convert_voltage(uint16_t data) {
    float scale = 3.3/4096.0;
    return ((float)(data))*scale;
}

