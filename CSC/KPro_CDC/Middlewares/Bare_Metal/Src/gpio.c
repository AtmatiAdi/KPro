/*
 * gpio.c
 *
 *  Created on: Jun 24, 2021
 *      Author: atmat
 */

#include "gpio.h"

void GPIOC_MODER_pc4_pc5_uart_init(){
	     // Active alternate function in GPIO port
	    //	These bits are written by software to configure the I/O mode.
	   //		00: Input mode (reset state)
	  //		01: General purpose output mode
	 //		10: Alternate function mode
	//		11: Analog mode
	GPIOC->MODER &= ~(GPIO_MODER_MODER5_Msk | GPIO_MODER_MODER4_Msk);
	// Set MODER as alternate function
	GPIOC->MODER |= (0b10 << GPIO_MODER_MODER4_Pos | 0b10 << GPIO_MODER_MODER5_Pos);
	// Alternate function mapping
	GPIOC->AFR[0] |= (GPIO_AF7_USART1 << GPIO_AFRL_AFRL4_Pos | GPIO_AF7_USART1 << GPIO_AFRL_AFRL5_Pos );
	    // Configure full speed
	   //	These bits are written by software to configure the I/O output speed.
	  //	x0: Low speed
	 //		01: Medium speed
	//		11: High speed
	GPIOC->OSPEEDR |= ((0b11 << GPIO_OSPEEDER_OSPEEDR4_Pos) | (0b11 << GPIO_OSPEEDER_OSPEEDR5_Pos));
}

void GPIO_MODER_set_mode(char port, uint16_t pin, uint8_t mode){
	switch(port){
	case 'A':{
		GPIOA->MODER &= ~(0b11 << pin);
		GPIOA->MODER |=  (mode << pin);
		break;
	}
	case 'B':{
		GPIOB->MODER &= ~(0b11 << pin);
		GPIOB->MODER |=  (mode << pin);
		break;
	}
	case 'C':{
		GPIOC->MODER &= ~(0b11 << pin);
		GPIOC->MODER |=  (mode << pin);
		break;
	}
	case 'D':{
		GPIOD->MODER &= ~(0b11 << pin);
		GPIOD->MODER |=  (mode << pin);
		break;
	}
	case 'E':{
		GPIOE->MODER &= ~(0b11 << pin);
		GPIOE->MODER |=  (mode << pin);
		break;
	}
	case 'F':{
		GPIOF->MODER &= ~(0b11 << pin);
		GPIOF->MODER |=  (mode << pin);
		break;
	}
	}

}
