/*
 * print.c
 *
 *  Created on: May 9, 2020
 *      Author: FLL1GA
 */
#include "print.h"
#include "miniprintf.h"
#include "uc_uart.h"

void print_setup(void){

	uart_setup();
}

void print(const char *format, ...){
    va_list args;

    va_start(args, format);
    mini_vprintf_cooked(uart_putc, format, args);
    va_end(args);
}

