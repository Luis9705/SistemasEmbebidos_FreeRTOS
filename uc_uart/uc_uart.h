/*
 * uc_uart.h
 *
 *  Created on: May 2, 2020
 *      Author: FLL1GA
 */

#ifndef UC_UART_H_
#define UC_UART_H_

void uart_setup(void);
void uart_putc(char ch);
void uart_puts(char str[]);
int uart_printf(const char *format, ...);


#endif /* UC_UART_H_ */
