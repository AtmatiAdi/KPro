/*
 * clock.cpp
 *
 *  Created on: 9 cze 2021
 *      Author: atmat
 */

#include "clock.h"

void FLASH_ACR_clock_init(){
	//Reset values in FLASH_ACR register
	FLASH->ACR = 0UL;
	// 000: Zero wait state, if 0 < HCLK ≤ 24 MHz
	// 001: One wait state, if 24 MHz < HCLK ≤ 48 MHz
	// 010: Two wait sates, if 48 < HCLK ≤ 72
	// Half-cycle access cannot be used when there is a prescaler different from 1 on the AHB clock.
	// The prefetch buffer must be kept on when using a prescaler different from 1 on the AHB clock
	// Instruction fetch: Prefetch buffer enabled for a faster CPU execution.
	// FLASH_ACR_HLFCYA < Flash Half Cycle Access Enable <-- not working
	// FLASH_ACR_PRFTBE < Prefetch Buffer Enable
	FLASH->ACR |= (FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2);
}

void RCC_HSE_enable(){
	// HSE oscillator ON
	RCC->CR |= RCC_CR_HSEON;
	// The HSERDY flag indicates if the HSE oscillator is stable or not
	while (!(RCC->CR & RCC_CR_HSERDY)) {};
}

void RCC_CFGR_pll_init(){
	//>>__PLL CONFIGURATION__<<
	// Disable the PLL by setting PLLON to 0
	RCC->CR &= ~(RCC_CR_PLLON);
	// Wait until PLLRDY is set. The PLL is now fully stopped.
	while ((RCC->CR & RCC_CR_PLLRDY)) {};
	// RCC_CFGR_USBPRE_DIV1_5 < USB prescaler is PLL clock divided by 1.5
	// RCC_CFGR_PLLMUL9 < PLL input clock*9
	// RCC_CFGR_PLLXTPRE_HSE_PREDIV_DIV2 < HSE/PREDIV clock divided by 2 for PLL entry
	// RCC_CFGR_PLLSRC_HSE_PREDIV < HSE/PREDIV clock selected as PLL entry clock source
	// RCC_CFGR_PPRE1_DIV2 < HCLK divided by 2
	RCC->CFGR &= ~(RCC_CFGR_USBPRE | RCC_CFGR_PLLMUL | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLSRC | RCC_CFGR_PPRE1 );
	RCC->CFGR |= (RCC_CFGR_USBPRE_DIV1_5 | RCC_CFGR_PLLMUL9 | RCC_CFGR_PLLXTPRE_HSE_PREDIV_DIV2 | RCC_CFGR_PLLSRC_HSE_PREDIV | RCC_CFGR_PPRE1_DIV2 );
	// Enable the PLL by setting PLLON to 1
	RCC->CR |= (RCC_CR_PLLON);
	// Wait until PLLRDY is cleared. The PLL is now running.
	while (!(RCC->CR & RCC_CR_PLLRDY)) {};
	//Select PLL as main clock source
	RCC->CFGR  &= ~(RCC_CFGR_SW);
	// RCC_CFGR_SW_PLL < PLL selected as system clock */
	RCC->CFGR  |=  (RCC_CFGR_SW_PLL);
	// Wait until PLL is selected as main clock source
	while (!(RCC->CFGR & RCC_CFGR_SWS_PLL)) {};
}

void RCC_APB1ENR_enable(){
	// APB1 peripheral clock enable register
	// Before using a peripheral user has to enable its clock in the RCC_AHBENR, RCC_APB2ENR or RCC_APB1ENR register.
	// RCC_APB1ENR_PWREN < PWR clock enable
	RCC->APB1ENR |= (RCC_APB1ENR_PWREN);
	//RCC->APB2ENR |= (RCC_APB1ENR_PWREN);
}

void RCC_APB1ENR_usb_clock_enable(){
	// APB1 peripheral clock enable register
	// Before using a peripheral user has to enable its clock in the RCC_AHBENR, RCC_APB2ENR or RCC_APB1ENR register.
	// RCC_APB1ENR_USBEN < USB clock enable
	RCC->APB1ENR |= (RCC_APB1ENR_USBEN);
	//RCC->APB2ENR |= (RCC_APB1ENR_PWREN);
}

void RCC_APB2ENR_usart_clock_enable(){
		// Enable Peripheral APB2 Clock
	   //		USART1EN: USART1 clock enable
	  //		Set and cleared by software.
	 //		0: USART1 clock disabled
	//		1: USART1 clock enabled
	RCC->APB2ENR |= (RCC_APB2ENR_USART1EN);
}

void RCC_AHBENR_port_enable(char port){
	switch(port){
	case 'A':{
		RCC->AHBENR |= (RCC_AHBENR_GPIOAEN);
		break;
	}
	case 'B':{
		RCC->AHBENR |= (RCC_AHBENR_GPIOBEN);
		break;
	}
	case 'C':{
			// Enable Port clock
		   //		IOPCEN: I/O port C clock enable
		  //		Set and cleared by software.
		 //		0: I/O port C clock disabled
		//		1: I/O port C clock enabled
		RCC->AHBENR |= (RCC_AHBENR_GPIOCEN);
	break;
	}
	case 'D':{
		RCC->AHBENR |= (RCC_AHBENR_GPIODEN);
		break;
	}
	case 'E':{
		RCC->AHBENR |= (RCC_AHBENR_GPIOEEN);
		break;
	}
	case 'F':{
		RCC->AHBENR |= (RCC_AHBENR_GPIOFEN);
		break;
	}
	}

}

void RCC_AHBENR_dma_clock_enable(){
	// DMA1 Clock enable
	RCC->AHBENR |= (RCC_AHBENR_DMA1EN);
}

void RCC_APB2ENR_tim17_clock_enable(){
	RCC->APB2ENR |= (RCC_APB2ENR_TIM17EN);
}
