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

void Clock_Flash_Init(void);
void HSE_Enable(void);
void PLL_Init(void);
void APB_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* BARE_METAL_CLOCK_H_ */
