/*
 * uc_uart.c
 *
 *  Created on: May 2, 2020
 *      Author: FLL1GA
 */
/// @file uc_uart.c
/// @brief Functions to use UART

#include "uc_uart.h"	///<
#include "usart.h"
#include "miniprintf.h"
#include "main.h"
#include "string.h"
#include <stdbool.h>

UART_HandleTypeDef * huart; ///<UART Handler

bool uart_enabled = false; ///<Variable to know

/**
 * @brief Sets up the UART.
 * @retval None
 */
void uart_setup(void){
	if(!uart_enabled){
		MX_USART1_UART_Init();
		huart = &huart1;
		uart_enabled = true;
	}

}

/**
 * @brief Sets up the UART.
 * @retval None
 */
void uart_enable_RX_IT(void){
	if(!uart_enabled){
		MX_USART1_UART_Init();
		huart = &huart1;
		uart_enabled = true;
	}
	MODIFY_REG(huart->Instance->CR1,
	             (uint32_t)(USART_CR1_RXNEIE),
	             (1<<5));
	HAL_NVIC_SetPriority(USART1_IRQn, 5, 5);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/**
 * @brief UART putc function.
 * @param[in] ch Char for putc function
 * @retval None
 */
void uart_putc(char ch){

	unsigned char buff[1] = {ch};
	HAL_UART_Transmit(huart, (uint8_t *) buff, 1, 10);
}

/**
 * @brief UART puts function.
 * @param[in] str Char array for puts function
 * @retval None
 */
void uart_puts(char str[]){
	HAL_UART_Transmit(huart, (uint8_t *) str, strlen(str), 10);
}

/**
 * @brief UART printf function.
 * @param[in] format
 * @return rc
 */
int uart_printf(const char *format, ...){
    va_list args;
    int rc;

    va_start(args, format);
    rc = mini_vprintf_cooked(uart_putc, format, args);
    va_end(args);
    return rc;
}
