/*
 * temp_sensor.c
 *
 *  Created on: May 2, 2020
 *      Author: FLL1GA
 */


/// @file temp_sensor.c
//  Copyright 2020 Copyright Equipo 2

#include "temp_sensor.h"
#include "main.h"

/**
 * Sets up the ADC peripherals needed for the temperature sensor.
 */
void temp_sensor_setup(void * init_struct) {
    adc_pin_setup();
    adc_setup(init_struct);
}

/**
 * Reads the ADC value and returns the temperature in degrees.
 * @param[out] temperature
 */
uint16_t temp_sensor_read(void) {
    uint16_t adc_data = adc_read();
    return (uint16_t)(adc_convert_voltage(adc_data)*100.0);
}
