/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "uart.h"
#include "clock.h"
#include "gpio.h"
#include "dma.h"
#include "tim.h"
//#include "stm32f3xx_it.h"
//#include "UART_DMA.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define SEG_A	1
#define SEG_B	2
#define SEG_C	4
#define SEG_D	8
#define SEG_E	16
#define SEG_F	32
#define SEG_G	64
#define SEG_H	128
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
char UART_Received[1];
uint8_t UART_RecFlag = 0;
char msg[3] = {'X','D','D'};

void my_USART1_IRQHandler(){
	//uint32_t isr = USART1->ISR;
	//uint32_t cr1 = USART1->CR1;
	if (USART1->ISR & USART_ISR_RXNE_Msk){
		UART_RecFlag = 1;
		UART_Received[0] = USART1->RDR;
		NVIC_ClearPendingIRQ(USART1_IRQn);
	}
	NVIC_ClearPendingIRQ(USART1_IRQn);
}

void my_DMA1_Channel4_IRQHandler(){
	if (DMA1->ISR & DMA_ISR_TCIF4){
		DMA1->IFCR |= (DMA_IFCR_CTCIF4);
	}
	// Disable DMA channel
	DMA1_Channel4->CCR &= ~(DMA_CCR_EN);
	NVIC_ClearPendingIRQ(DMA1_Channel4_IRQn);
}

uint8_t SegSel = 0;
uint8_t Seg[4] = {0,0,0,0};
void my_TIM1_TRG_COM_TIM17_IRQHandler(){
	if (TIM17->SR & TIM_SR_UIF){
		TIM17->SR &= ~(TIM_SR_UIF);
		//HAL_GPIO_TogglePin(LED_5V_GPIO_Port, LED_5V_Pin);
		DispNum(Seg[SegSel],SegSel);
		++SegSel;
		if(SegSel > 3) SegSel = 0;
	}
	NVIC_ClearPendingIRQ(TIM17_IRQn);
}

void DispVal(uint32_t val){
	if (val < 10000){
		Seg[3] = val % 10;
		val = val / 10;
		Seg[2] = val % 10;
		val = val / 10;
		Seg[1] = val % 10;
		val = val / 10;
		Seg[0] = val % 10 + 10;
	} else if (val < 100000){
		val = val / 10;
		Seg[3] = val % 10;
		val = val / 10;
		Seg[2] = val % 10;
		val = val / 10;
		Seg[1] = val % 10 + 10;
		val = val / 10;
		Seg[0] = val % 10;
	} else if (val < 1000000){
		val = val / 100;
		Seg[3] = val % 10;
		val = val / 10;
		Seg[2] = val % 10 + 10;
		val = val / 10;
		Seg[1] = val % 10;
		val = val / 10;
		Seg[0] = val % 10;
	} else if (val < 10000000){
		val = val / 1000;
		Seg[3] = val % 10 + 10;
		val = val / 10;
		Seg[2] = val % 10;
		val = val / 10;
		Seg[1] = val % 10;
		val = val / 10;
		Seg[0] = val % 10;
	}
}

void DispNum(uint8_t num, uint8_t disp){
	uint8_t IsDot = 0;
	HAL_GPIO_WritePin(SEG_1_GPIO_Port, SEG_1_Pin, 0);
	HAL_GPIO_WritePin(SEG_1__GPIO_Port, SEG_1__Pin, 0);
	HAL_GPIO_WritePin(SEG_1__GPIO_Port, SEG_1___Pin, 0);

	HAL_GPIO_WritePin(SEG_2_GPIO_Port, SEG_2_Pin, 0);
	HAL_GPIO_WritePin(SEG_2__GPIO_Port, SEG_2__Pin, 0);

	HAL_GPIO_WritePin(SEG_3_GPIO_Port, SEG_3_Pin, 0);
	HAL_GPIO_WritePin(SEG_3__GPIO_Port, SEG_3__Pin, 0);

	HAL_GPIO_WritePin(SEG_4_GPIO_Port, SEG_4_Pin, 0);
	HAL_GPIO_WritePin(SEG_4__GPIO_Port, SEG_4__Pin, 0);

	if(num > 9){
		num -= 10;
		IsDot = 1;
	}

	switch(disp){
	case 0:{
		HAL_GPIO_WritePin(SEG_1_GPIO_Port, SEG_1_Pin, 1);
		HAL_GPIO_WritePin(SEG_1__GPIO_Port, SEG_1__Pin, 1);
		HAL_GPIO_WritePin(SEG_1___GPIO_Port, SEG_1___Pin, 1);
		break;
	}
	case 1:{
		HAL_GPIO_WritePin(SEG_2_GPIO_Port, SEG_2_Pin, 1);
		HAL_GPIO_WritePin(SEG_2__GPIO_Port, SEG_2__Pin, 1);
		break;
	}
	case 2:{
		HAL_GPIO_WritePin(SEG_3_GPIO_Port, SEG_3_Pin, 1);
		HAL_GPIO_WritePin(SEG_3__GPIO_Port, SEG_3__Pin, 1);
		break;
	}
	case 3:{
		HAL_GPIO_WritePin(SEG_4_GPIO_Port, SEG_4_Pin, 1);
		HAL_GPIO_WritePin(SEG_4__GPIO_Port, SEG_4__Pin, 1);
		break;
	}
	}

	switch(num){
	case 0:{
		SetDisp((uint8_t)SEG_D | SEG_E);
		break;
	}
	case 1:{
		SetDisp((uint8_t)SEG_A | SEG_B | SEG_D | SEG_H | SEG_F | SEG_E);
		break;
	}
	case 2:{
		SetDisp((uint8_t)SEG_B | SEG_E | SEG_G);
		break;
	}
	case 3:{
		SetDisp((uint8_t)SEG_B | SEG_H | SEG_E);
		break;
	}
	case 4:{
		SetDisp((uint8_t)SEG_A | SEG_H | SEG_F | SEG_E);
		break;
	}
	case 5:{
		SetDisp((uint8_t)SEG_C | SEG_H | SEG_E);
		break;
	}
	case 6:{
		SetDisp((uint8_t)SEG_C | SEG_E);
		break;
	}
	case 7:{
		SetDisp((uint8_t)SEG_B | SEG_D | SEG_H | SEG_F | SEG_E);
		break;
	}
	case 8:{
		SetDisp((uint8_t) SEG_E);
		break;
	}
	case 9:{
		SetDisp((uint8_t)SEG_H | SEG_E);
		break;
	}
	case 10:{
		SetDisp((uint8_t)0);
		break;
	}
	}
	if(IsDot) HAL_GPIO_WritePin(SEG_E_GPIO_Port, SEG_E_Pin, 0);
}

void SetDisp(uint8_t seg){
	HAL_GPIO_WritePin(SEG_A_GPIO_Port, SEG_A_Pin, (seg & (1 << 0)));
	HAL_GPIO_WritePin(SEG_B_GPIO_Port, SEG_B_Pin, (seg & (1 << 1)));
	HAL_GPIO_WritePin(SEG_C_GPIO_Port, SEG_C_Pin, (seg & (1 << 2)));
	HAL_GPIO_WritePin(SEG_D_GPIO_Port, SEG_D_Pin, (seg & (1 << 3)));
	HAL_GPIO_WritePin(SEG_E_GPIO_Port, SEG_E_Pin, (seg & (1 << 4)));
	HAL_GPIO_WritePin(SEG_F_GPIO_Port, SEG_F_Pin, (seg & (1 << 5)));
	HAL_GPIO_WritePin(SEG_G_GPIO_Port, SEG_G_Pin, (seg & (1 << 6)));
	HAL_GPIO_WritePin(SEG_H_GPIO_Port, SEG_H_Pin, (seg & (1 << 7)));
}
//UARTDMA_HandleTypeDef huartdma;
char ParseBuffer[8];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  	// CONFIGURE CLOCK
	FLASH_ACR_clock_init();
	RCC_HSE_enable();
	RCC_CFGR_pll_init();
	RCC_APB1ENR_enable();
	RCC_APB1ENR_usb_clock_enable();

	// HALL INITALIZERS
		MX_GPIO_Init();
		MX_USB_DEVICE_Init();

	// USART INITALIZATION
	RCC_APB2ENR_usart_clock_enable();
	RCC_AHBENR_port_enable('C');
	GPIOC_MODER_pc4_pc5_uart_init();
	USART1_CR1_disable_parity();
	USART1_CR1_world_8b();
	USART1_CR1_oversampling_16();
	USART1_BBR_set_baundrate(72000000, 9600);
	USART1_CR2_stop_1bit();
	USART1_CR1_enable_RXNEIE();
	USART1_CR1_enable_receiver();
	USART1_CR3_enable_dma();
	USART1_CR1_enable_transmitter();
	USART1_CR1_enable();
	USART1_ICR_clear_tansfer_completec_flag();

	// DMA FOR USART INITRALIZATION
	RCC_AHBENR_dma_clock_enable();
	DMA1_Channel4_CCR_uart_init();
	DMA1_Channel4_CPAR_set_tdr_buffer();

	NVIC_SetPriority(USART1_IRQn, 6);
	NVIC_EnableIRQ(USART1_IRQn);

	NVIC_SetPriority(DMA1_Channel4_IRQn, 0);
	NVIC_EnableIRQ(DMA1_Channel4_IRQn);

	// TIMER 17 INITALIZATION
	RCC_APB2ENR_tim17_clock_enable();
	NVIC_SetPriority(TIM17_IRQn, 3);
	NVIC_EnableIRQ(TIM17_IRQn);
	TIM17_CR1_reset_timer17();
	TIM17_PSC_set_presc_period(72000 - 1, 20 -1);
	TIM17_DIER_enable_update_interrupt();
	TIM17_CR1_enable();

	HAL_GPIO_WritePin(VOUT_EN_GPIO_Port, VOUT_EN_Pin, 0);
	HAL_GPIO_WritePin(V_SEL_GPIO_Port, V_SEL_Pin, 1);
	HAL_GPIO_WritePin(LED_5V_GPIO_Port, LED_5V_Pin, 0);

	while(1){
		if (UART_RecFlag){
			// After reciving data from UART send it throught USB
			UART_RecFlag = 0;
			CDC_Transmit_FS(&UART_Received, 1);
		}
	}

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_FATFS_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 0;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 65535;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, SEG_D_Pin|SEG_B_Pin|SEG_A_Pin|SEG_C_Pin
                          |SEG_4__Pin|SEG_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(V_SEL_GPIO_Port, V_SEL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_5V_Pin|VOUT_EN_Pin|SEG_1__Pin|SEG_1_Pin
                          |SEG_2__Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SEG_E_Pin|SEG_G_Pin|SEG_F_Pin|SEG_H_Pin
                          |SEG_1___Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USB_TX_DIODE_Pin|USB_RX_DIODE_Pin|SEG_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SEG_3__Pin|SEG_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SEG_D_Pin SEG_B_Pin SEG_A_Pin SEG_C_Pin
                           SEG_4__Pin SEG_4_Pin */
  GPIO_InitStruct.Pin = SEG_D_Pin|SEG_B_Pin|SEG_A_Pin|SEG_C_Pin
                          |SEG_4__Pin|SEG_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : V_SEL_Pin */
  GPIO_InitStruct.Pin = V_SEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(V_SEL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_5V_Pin VOUT_EN_Pin SEG_1__Pin SEG_1_Pin
                           SEG_2__Pin */
  GPIO_InitStruct.Pin = LED_5V_Pin|VOUT_EN_Pin|SEG_1__Pin|SEG_1_Pin
                          |SEG_2__Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG_E_Pin SEG_G_Pin SEG_F_Pin SEG_H_Pin
                           SEG_1___Pin */
  GPIO_InitStruct.Pin = SEG_E_Pin|SEG_G_Pin|SEG_F_Pin|SEG_H_Pin
                          |SEG_1___Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DET_Pin */
  GPIO_InitStruct.Pin = DET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_TX_DIODE_Pin USB_RX_DIODE_Pin SEG_2_Pin */
  GPIO_InitStruct.Pin = USB_TX_DIODE_Pin|USB_RX_DIODE_Pin|SEG_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG_3__Pin SEG_3_Pin */
  GPIO_InitStruct.Pin = SEG_3__Pin|SEG_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
