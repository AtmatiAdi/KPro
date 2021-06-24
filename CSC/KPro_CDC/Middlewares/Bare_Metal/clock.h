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

void FLASH_ACR_clock_init(void);
void RCC_HSE_enable(void);
void RCC_CFGR_pll_init(void);
void RCC_APB1ENR_enable(void);

#ifdef __cplusplus
}
#endif

#endif /* BARE_METAL_CLOCK_H_ */
