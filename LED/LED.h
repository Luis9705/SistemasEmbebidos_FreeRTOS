/*
 * LED.h
 *
 *  Created on: May 9, 2020
 *      Author: FLL1GA
 */
/// @file LED.h
/// @brief LED Header file.
#ifndef LED_H_
#define LED_H_

typedef enum  {
  LED_ON, //!< LED_ON
  LED_OFF,//!< LED_OFF
} led_statusType;

void LED_setup(void);

void sensingLED_ON(void);
void sensingLED_OFF(void);
void sensingLED_TOGGLE(void);
led_statusType getSensingLED_STATUS(void);

void maxTempLED_ON(void);
void maxTempLED_OFF(void);
void maxTempLED_TOGGLE(void);
led_statusType getMaxLED_STATUS(void);

void minTempLED_ON(void);
void minTempLED_OFF(void);
void minTempLED_TOGGLE(void);
led_statusType getMinLED_STATUS(void);

#endif /* LED_H_ */
