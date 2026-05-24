/*
 * charger_uart.h
 *
 *  Created on: 24 May 2026
 *      Author: amrlxyz
 */

#ifndef INC_CHARGER_UART_H_
#define INC_CHARGER_UART_H_

void uart_init(void);

int printfDma(const char *format, ...);

//void printfFlushBuffer(void);


#endif /* INC_CHARGER_UART_H_ */
