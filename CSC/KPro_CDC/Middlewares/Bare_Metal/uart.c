/*
 * uart.c
 *
 *  Created on: Jun 24, 2021
 *      Author: atmat
 */

#include "uart.h"

void USART1_CR1_disable_parity(){
		// Configure Parity
	   //	This bit selects the hardware parity control (generation and detection)
	  //	0: Parity control disabled
	 //		1: Parity control enabled
	//		This bit field can only be written when the USART is disabled (UE=0)
	USART1->CR1 &= ~(USART_CR1_PCE_Msk);
}

void USART1_CR1_world_8b(){
	 	 // Configure word length
	    //	This bit, with bit 12 (M0), determines the word length. It is set or cleared by software.
	   //	M[1:0] = 00: 1 Start bit, 8 data bits, n stop bits
	  //	M[1:0] = 01: 1 Start bit, 9 data bits, n stop bits
	 //		M[1:0] = 10: 1 Start bit, 7 data bits, n stop bits
	//		This bit can only be written when the USART is disabled (UE=0).
	USART1->CR1 &= ~(USART_CR1_M0_Msk);
}

void USART1_CR1_oversampling_16(){
   	   // Configure oversampling
	  //		0: Oversampling by 16
	 //		1: Oversampling by 8
	//		This bit can only be written when the USART is disabled (UE=0).
	USART1->CR1 &= ~(USART_CR1_OVER8_Msk);
}

void USART1_BBR_set_baundrate(uint32_t clock, uint32_t rate){
	// Calculate baund rate
	uint32_t uartdiv = clock/rate;
	// calculate BRR
	USART1->BRR = (((uartdiv/16) << USART_BRR_DIV_MANTISSA_Pos) | ((uartdiv%16) << USART_BRR_DIV_FRACTION_Pos));
}

void USART1_CR2_stop_1bit(){
		  // Configure stop bits
	     //	These bits are used for programming the stop bits.
	    //	00: 1 stop bit
	   //	01: 0.5 stop bit
	  //	10: 2 stop bits
	 //		11: 1.5 stop bits
	//		This bit field can only be written when the USART is disabled (UE=0).
	USART1->CR2 &= ~(USART_CR2_STOP_Msk);			// Reset state -> 1 stop bit
}

void USART1_CR1_enable_RXNEIE(){
	   // Enable RXNEIE interrupt
	  //	This bit is set and cleared by software.
	 //		0: Interrupt is inhibited
	//		1: A USART interrupt is generated whenever ORE=1 or RXNE=1 in the USART_ISR register
	USART1->CR1 |= (USART_CR1_RXNEIE);			// Enable RXNEIE interrupt
}

void USART1_CR1_enable_receiver(){
	   // Enable receiving
	  //	This bit enables the receiver. It is set and cleared by software.
	 //		0: Receiver is disabled
	//		1: Receiver is enabled and begins searching for a start bit
	USART1->CR1 |= (USART_CR1_RE);
}

void USART1_CR3_enable_dma(){
	   // Enable DMA transmitter
	  //		This bit is set/reset by software
	 //		1: DMA mode is enabled for transmission
	//		0: DMA mode is disabled for transmission
	USART1->CR3 |= (USART_CR3_DMAT);
}

void USART1_CR1_enable_transmitter(){
	   // Enable Transmitter
	  //		This bit enables the transmitter. It is set and cleared by software.
	 //		0: Transmitter is disabled
	//		1: Transmitter is enabled
	USART1->CR1 |= (USART_CR1_TE);
}

void USART1_CR1_enable(){
		// Enable USART1
	   //		0: USART prescaler and outputs disabled, low-power mode
	  //		1: USART enabled
	 //		The DMA requests are also reset when UE = 0 so the DMA channel must be disabled
	//		before resetting the UE bit.
	USART1->CR1 |= (USART_CR1_UE);
}

void USART1_ICR_clear_tansfer_completec_flag(){
	  // USART clear TC transfer complete flag
	  USART1->ICR &= ~(USART_ICR_TCCF);
}
