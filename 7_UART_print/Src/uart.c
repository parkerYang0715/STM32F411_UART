/*  uart.c
 *  Created on: Jul 3, 2021
 *  Author: Parker Yang
 */
#include"uart.h"
#define GPIOAEN        (1U<<0)
#define UART2EN        (1U<<17)

#define CR1_TE         (1U<<3)
#define CR1_UE         (1U<<13)
#define SR_TXE         (1U<<7)

#define SYS_FREQ       (16000000)
#define APB1_CLK       SYS_FREQ

#define UART_BAUDRATE  (9600)

static void UART_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate);
static uint16_t compute_UART_bd(uint32_t PeriphClk, uint32_t BaudRate);


void UART2_write(int ch);
int __io_putchar(int ch){
	UART2_write(ch);
	return ch;
}


void UART_INIT(){
	/* configure UART GPIO PIN*/
	/* Enable clock access to GPIOA*/
	RCC->AHB1ENR |= GPIOAEN;

	/* set PA2 to alternate function mode */
	GPIOA->MODER &= ~(1U<<4);
	GPIOA->MODER |= (1U<<5);

	/* set PA2 alternate function type to UART TX (AF07) */
	GPIOA->AFR[0] |= (1U<<8);
	GPIOA->AFR[0] |= (1U<<9);
	GPIOA->AFR[0] |= (1U<<10);
	GPIOA->AFR[0] &= ~(1U<<11);


	/* configure UART module */
	/* Enable clock access to UART2 */
	RCC->APB1ENR |= UART2EN;

	/* configure Baudrate */
	UART_set_baudrate(USART2,APB1_CLK,UART_BAUDRATE);

	/* configure transfer direction */
	USART2->CR1 = CR1_TE;  // clear other bits

	/* Enable UART module */
	USART2->CR1 |= CR1_UE;
}

void UART2_write(int ch)
{
	/* Make sure the transmit data register is empty */
	while (!(USART2->SR & SR_TXE)){

	}

	/* Write to transmit data register */
	USART2->DR = (ch & 0xFF);
}
static void UART_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate)
{
	USARTx->BRR = compute_UART_bd(PeriphClk, BaudRate);
	// A baud rate register (USART_BRR) - 12-bit mantissa and 4-bit fraction
}
static uint16_t compute_UART_bd(uint32_t PeriphClk, uint32_t BaudRate)
{
	return (PeriphClk + (BaudRate/2U))/BaudRate;

}
