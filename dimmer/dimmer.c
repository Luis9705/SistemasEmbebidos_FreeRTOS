/*
 * dimmer.c
 *
 *  Created on: May 2, 2020
 *      Author: FLL1GA
 */

/// @file dimmer.c
/// @brief Dimmer function files.
// Copyright 2020 Equipo 2
#include "dimmer.h"
#include "tim.h"
#include "main.h"

TIM_HandleTypeDef * htim;



static uint16_t get_pwm_percentage_counts(uint16_t period, uint16_t percentage);
static void setPWM(TIM_HandleTypeDef timer, uint32_t channel, uint16_t period, uint16_t pulse);

/**
 * @brief Initializes the timer, sets it up as a PWM and starts the timer.
 * @retval None
 */
void dimmer_setup(void) {

	MX_TIM2_Init();
	htim = &htim2;
}

/**
 * @brief Updates the dimming intensity of the light.
 * @param[in] percentage
 * @retval None
 */
void dimmer_update_percentage(uint16_t percentage) {

	uint16_t pwm_period =  SystemCoreClock/PWM_PRESCALER/PWM_FREQUENCY;
	uint16_t pulse = get_pwm_percentage_counts(pwm_period, percentage);


	setPWM(*htim, TIM_CHANNEL_2, pwm_period, pulse);

}

/**
 * @brief Calculates a percentage of dimming given a constant.
 * @param[in] percentage
 * @param[out] counts
 * @return counts
 */
static uint16_t get_pwm_percentage_counts(uint16_t period, uint16_t percentage) {
    uint16_t counts = (percentage*period) / 100;
    if (counts == 0) {
        counts++;  //  this prevents the counter to have a negative value when
    // we substract 1
    }
    return counts;
}

/**
 * @brief Sets PWM used to dim the LED.
 * @param[in] timer
 * @param[in] channel
 * @param[in] period
 * @param[in] pulse
 */
static void setPWM(TIM_HandleTypeDef timer, uint32_t channel, uint16_t period, uint16_t pulse)
{
 HAL_TIM_PWM_Stop(&timer, channel); // stop generation of pwm
 TIM_OC_InitTypeDef sConfigOC;
 timer.Init.Period = period; // set the period duration
 HAL_TIM_PWM_Init(&timer); // reinititialise with new period value

 sConfigOC.OCMode = TIM_OCMODE_PWM2;
 sConfigOC.Pulse = pulse;
 sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
 sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
 HAL_TIM_PWM_ConfigChannel(&timer, &sConfigOC, channel);
 HAL_TIM_PWM_Start(&timer, channel); // start pwm generation
}

