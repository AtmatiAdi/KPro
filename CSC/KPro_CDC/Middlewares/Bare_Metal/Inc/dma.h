/*
 * dma.h
 *
 *  Created on: Jun 24, 2021
 *      Author: atmat
 */

#ifndef BARE_METAL_DMA_H_
#define BARE_METAL_DMA_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f3xx_hal.h"

void DMA1_Channel4_CCR_uart_init(void);
void DMA1_Channel4_CPAR_set_tdr_buffer(void);

#ifdef __cplusplus
}
#endif

#endif /* BARE_METAL_DMA_H_ */
