/*
 * gpio.h
 *
 *  Created on: Jun 24, 2021
 *      Author: atmat
 */

#ifndef BARE_METAL_GPIO_H_
#define BARE_METAL_GPIO_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f3xx_hal.h"

void GPIOC_MODER_pc4_pc5_uart_init(void);
void GPIO_MODER_set_mode(char port, uint16_t pin, uint8_t mode);


#ifdef __cplusplus
}
#endif

#endif /* BARE_METAL_GPIO_H_ */
