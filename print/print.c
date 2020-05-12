/*
 * print.c
 *
 *  Created on: May 9, 2020
 *      Author: FLL1GA
 */
/// @file print.h
/// @brief Print functions.
#include "print.h"
#include "miniprintf.h"
#include "uc_uart.h"

/**
 * @brief Sets up the device used to print info.
 * @retval None
 */
void print_setup(void){

	uart_setup();
}

/**
 * @brief Print function used.
 * @param format
 * @retval None
 */
void print(const char *format, ...){
    va_list args;

    va_start(args, format);
    mini_vprintf_cooked(uart_putc, format, args);
    va_end(args);
}

