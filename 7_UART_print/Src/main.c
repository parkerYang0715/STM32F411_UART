#include<stdio.h>
#include<stdint.h>
#include "stm32f4xx.h"
#include"uart.h"
// USART2 connects to APB1
// RCC APB1 peripheral clock enable register (RCC_APB1ENR)
// set bit 17 to enableUSART2

int main()
{
	UART_INIT();
	while(1){
		UART2_write('P');
		UART2_write('A');
		UART2_write('R');
		UART2_write('K');
		UART2_write('E');
		UART2_write('R');
		printf("\nHello STM32F%d",411);
	}

}
