/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void my_USART1_IRQHandler();
void my_DMA1_Channel4_IRQHandler();
void my_TIM1_TRG_COM_TIM17_IRQHandler();
void SetDisp(uint8_t seg);
void DispNum(uint8_t num, uint8_t disp);
void DispVal(uint32_t val);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SEG_D_Pin GPIO_PIN_2
#define SEG_D_GPIO_Port GPIOE
#define SEG_B_Pin GPIO_PIN_3
#define SEG_B_GPIO_Port GPIOE
#define SEG_A_Pin GPIO_PIN_5
#define SEG_A_GPIO_Port GPIOE
#define SEG_C_Pin GPIO_PIN_6
#define SEG_C_GPIO_Port GPIOE
#define V_SEL_Pin GPIO_PIN_10
#define V_SEL_GPIO_Port GPIOF
#define LED_5V_Pin GPIO_PIN_0
#define LED_5V_GPIO_Port GPIOC
#define VOUT_EN_Pin GPIO_PIN_1
#define VOUT_EN_GPIO_Port GPIOC
#define SEG_E_Pin GPIO_PIN_0
#define SEG_E_GPIO_Port GPIOA
#define SEG_G_Pin GPIO_PIN_1
#define SEG_G_GPIO_Port GPIOA
#define SEG_F_Pin GPIO_PIN_2
#define SEG_F_GPIO_Port GPIOA
#define SEG_H_Pin GPIO_PIN_3
#define SEG_H_GPIO_Port GPIOA
#define DET_Pin GPIO_PIN_11
#define DET_GPIO_Port GPIOB
#define USB_TX_DIODE_Pin GPIO_PIN_8
#define USB_TX_DIODE_GPIO_Port GPIOD
#define USB_RX_DIODE_Pin GPIO_PIN_9
#define USB_RX_DIODE_GPIO_Port GPIOD
#define SEG_1___Pin GPIO_PIN_15
#define SEG_1___GPIO_Port GPIOA
#define SEG_1__Pin GPIO_PIN_10
#define SEG_1__GPIO_Port GPIOC
#define SEG_1_Pin GPIO_PIN_11
#define SEG_1_GPIO_Port GPIOC
#define SEG_2__Pin GPIO_PIN_12
#define SEG_2__GPIO_Port GPIOC
#define SEG_2_Pin GPIO_PIN_0
#define SEG_2_GPIO_Port GPIOD
#define SEG_3__Pin GPIO_PIN_8
#define SEG_3__GPIO_Port GPIOB
#define SEG_3_Pin GPIO_PIN_9
#define SEG_3_GPIO_Port GPIOB
#define SEG_4__Pin GPIO_PIN_0
#define SEG_4__GPIO_Port GPIOE
#define SEG_4_Pin GPIO_PIN_1
#define SEG_4_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
