/*
 * uc_uart.c
 *
 *  Created on: May 2, 2020
 *      Author: FLL1GA
 */

#include "uc_uart.h"
#include "miniprintf.h"
#include "main.h"
#include "string.h"

UART_HandleTypeDef * huart;

void uart_setup(void * init_struct){
	huart = (UART_HandleTypeDef *)init_struct;
}

void uart_putc(char ch){

	unsigned char buff[1] = {ch};
	HAL_UART_Transmit(huart, (uint8_t *) buff, 1, 10);
}
void uart_puts(char str[]){
	HAL_UART_Transmit(huart, (uint8_t *) str, strlen(str), 10);
}
int uart_printf(const char *format, ...){
    va_list args;
    int rc;

    va_start(args, format);
    rc = mini_vprintf_cooked(uart_putc, format, args);
    va_end(args);
    return rc;
}
