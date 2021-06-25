/*
 * uart.h
 *
 *  Created on: Jun 24, 2021
 *      Author: atmat
 */

#ifndef BARE_METAL_UART_H_
#define BARE_METAL_UART_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f3xx_hal.h"

void USART1_CR1_disable_parity(void);
void USART1_CR1_world_8b(void);
void USART1_CR1_oversampling_16(void);
void USART1_BBR_set_baundrate(uint32_t clock, uint32_t rate);
void USART1_CR2_stop_1bit(void);
void USART1_CR1_enable_RXNEIE(void);
void USART1_CR1_enable_receiver(void);
void USART1_CR3_enable_dma(void);
void USART1_CR1_enable_transmitter(void);
void USART1_CR1_enable(void);
void USART1_ICR_clear_tansfer_completec_flag(void);

#ifdef __cplusplus
}
#endif

#endif /* BARE_METAL_UART_H_ */
