/*
 * timer.c
 *
 *  Created on: May 9, 2020
 *      Author: FLL1GA
 */

/// @file timer.c
/// @brief Functions to use the Timer.
///
#include "timer.h"
#include "tim.h"

/**
 * @brief Sets up the timer.
 * @retval None
 */
void timer_setup(void){
	  MX_TIM3_Init();
	  HAL_TIM_Base_Start_IT(&htim3);
}
