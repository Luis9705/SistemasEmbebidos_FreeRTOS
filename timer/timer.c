/*
 * timer.c
 *
 *  Created on: May 9, 2020
 *      Author: FLL1GA
 */

#include "timer.h"
#include "tim.h"

void timer_setup(void){
	  MX_TIM3_Init();
	  HAL_TIM_Base_Start_IT(&htim3);
}
