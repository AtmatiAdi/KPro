/*
 * clock.h
 *
 *  Created on: 9 cze 2021
 *      Author: atmat
 */

#ifndef BARE_METAL_CLOCK_H_
#define BARE_METAL_CLOCK_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f3xx_hal.h"

void FLASH_ACR_clock_init(void);
void RCC_HSE_enable(void);
void RCC_CFGR_pll_init(void);
void RCC_APB1ENR_enable(void);
void RCC_APB1ENR_usb_clock_enable(void);
void RCC_APB2ENR_usart_clock_enable(void);
void RCC_AHBENR_port_enable(char port);
void RCC_AHBENR_dma_clock_enable(void);
void RCC_APB2ENR_tim17_clock_enable(void);

#ifdef __cplusplus
}
#endif

#endif /* BARE_METAL_CLOCK_H_ */
