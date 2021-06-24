/*
 * dma.c
 *
 *  Created on: Jun 24, 2021
 *      Author: atmat
 */

#include "dma.h"

void DMA1_Channel4_CCR_uart_init(){
	  // DMA1 Clear control register
	  DMA1_Channel4->CCR = 0;
	  // Wait until DMA is disabled
	  while(DMA1_Channel4->CCR & (DMA_CCR_EN));
	     // Enable Transfer complete interrupt
	    //		This bit is set and cleared by software.
	   //		0: TC interrupt disabled
	  //		1: TC interrupt enabled
	  DMA1_Channel4->CCR |= (DMA_CCR_TCIE);
	     // Memory increment mode enable
	    //		This bit is set and cleared by software.
	   //		0: Memory increment mode disabled
	  //		1: Memory increment mode enabled
	  DMA1_Channel4->CCR |= (DMA_CCR_MINC);
	     // Transfer direction mem to peri
	    //		This bit is set and cleared by software.
	   //		0: Read from peripheral
	  //		1: Read from memory
	  DMA1_Channel4->CCR |= (DMA_CCR_DIR);
	       // Medium Priority
	      //	These bits are set and cleared by software.
	     //		00: Low
	    //		01: Medium
	   //		10: High
	  //		11: Very high
	  DMA1_Channel4->CCR |= (DMA_CCR_PL_0);
}

void DMA1_Channel4_CPAR_set_tdr_buffer(){
		// DMA peripheral USART TX buffer address
	   //		This register must not be written when the channel is enabled.
	  //		Base address of the peripheral data register from/to which the data will be read/written.
	 //		When PSIZE is 01 (16-bit), the PA[0] bit is ignored. Access is automatically aligned to a halfword address.
	//		When PSIZE is 10 (32-bit), PA[1:0] are ignored. Access is automatically aligned to a word address.
	DMA1_Channel4->CPAR = (uint32_t)&USART1->TDR;
}
