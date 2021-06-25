/*
 * tim.h
 *
 *  Created on: 25 cze 2021
 *      Author: atmat
 */

#ifndef BARE_METAL_INC_TIM_H_
#define BARE_METAL_INC_TIM_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f3xx_hal.h"

void TIM17_CR1_reset_timer17(void);
void TIM17_PSC_set_presc_period(uint32_t presc, uint32_t period);
void TIM17_DIER_enable_update_interrupt(void);
void TIM17_CR1_enable(void);

#ifdef __cplusplus
}
#endif

#endif /* BARE_METAL_INC_TIM_H_ */
