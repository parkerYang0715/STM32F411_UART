#include<stdio.h>
#include<stdint.h>
#include "stm32f4xx.h"
#include"uart.h"

/* LED setting */
#define GPIOAEN     (1U<<0)
#define PIN5        (1U<<5)
#define LED_PIN     PIN5
/* finish LED setting*/

char key;
int main()
{
	// PART1: set LED
	/* 1. Enable clock access to GPIOA */
	RCC->AHB1ENR |= GPIOAEN;

	/* 2. Set PA5 as output PIN */
	GPIOA->MODER |= (1U<<10);
	GPIOA->MODER &= ~(1U<<11);
	GPIOA->ODR |= LED_PIN; //
	// PART2: USART

	UART2_RXTX_INIT();
	while(1){
		key = UART2_read();
		if( key & 1 ){
			GPIOA->ODR |= LED_PIN;
		}
		else{
			GPIOA->ODR &= ~LED_PIN;
		}
	}

}
