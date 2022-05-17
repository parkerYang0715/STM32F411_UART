/*
 * uart.h
 *
 *  Created on: Jul 3, 2021
 *      Author: Parker Yang
 */

#ifndef UART_H_
#define UART_H_
#include<stdint.h>
#include "stm32f4xx.h"
void UART2_RXTX_INIT();
void UART2_TX_INIT();
char UART2_read();

#endif /* UART_H_ */
