/*
 * temp_sensor.h
 *
 *  Created on: May 2, 2020
 *      Author: FLL1GA
 */

#ifndef TEMP_SENSOR_H_
#define TEMP_SENSOR_H_


#include <stdint.h>

#define I2C_ADC

#ifdef I2C_ADC
#include "i2c_adc.h"
#else
#include "uc_adc.h"
#endif


void temp_sensor_setup(void * init_struct);
uint16_t temp_sensor_read(void);


#endif /* TEMP_SENSOR_H_ */
