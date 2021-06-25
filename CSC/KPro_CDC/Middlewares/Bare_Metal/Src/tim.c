/*
 * tim.c
 *
 *  Created on: 25 cze 2021
 *      Author: atmat
 */

#include "tim.h"

void TIM17_CR1_reset_timer17(){
	// Disbale timer
	TIM17->CR1 &= ~(TIM_CR1_CEN);
	// Reset timer
	RCC->APB2RSTR |= (RCC_APB2RSTR_TIM17RST);
	RCC->APB2RSTR &= ~(RCC_APB2RSTR_TIM17RST);
}

void TIM17_PSC_set_presc_period(uint32_t presc, uint32_t period){
	// Set prescaller
	TIM17->PSC = presc;
	TIM17->ARR = period;
	// Reset timer and apply settings
	TIM17->EGR |= (TIM_EGR_UG);
}

void TIM17_DIER_enable_update_interrupt(){
	// Enable update interrupt
	TIM17->DIER |= (TIM_DIER_UIE);
}

void TIM17_CR1_enable(){
	// Enable timer
	TIM17->CR1 |= TIM_CR1_CEN;
}
