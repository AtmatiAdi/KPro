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
#include "clock.h"
//#include "stm32f3xx_it.h"
//#include "UART_DMA.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
char UART_Received[1];
uint8_t UART_RecFlag = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	UART_RecFlag = 1;
	HAL_UART_Receive_IT(&huart1, &UART_Received, 1);
}
void my_USART1_IRQHandler(){
	if (USART1->ISR & USART_ISR_RXNE_Msk){
		UART_RecFlag = 1;
		UART_Received[0] = USART1->RDR;
		NVIC_ClearPendingIRQ(USART1_IRQn);
	}

	NVIC_ClearPendingIRQ(USART1_IRQn);
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

  //Clock_Flash_Init();
  //HSE_Enable();
  //PLL_Init();
  //APB_Init();


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  //MX_DMA_Init();
  MX_SPI2_Init();
  //MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  // Function below needs changes in DMA1_Channel5_IRQHandler and USART1_IRQHandler
  //UARTDMA_Init(&huartdma, &huart1);

  // Enable Peripheral APB2 Clock
  //		USART1EN: USART1 clock enable
  //		Set and cleared by software.
  //		0: USART1 clock disabled
  //		1: USART1 clock enabled
  RCC->APB2ENR |= (RCC_APB2ENR_USART1EN);
  // Enable Port clock
  //		IOPCEN: I/O port C clock enable
  //		Set and cleared by software.
  //		0: I/O port C clock disabled
  //		1: I/O port C clock enabled
  RCC->AHBENR |= (RCC_AHBENR_GPIOCEN);
  // Active alternate function in GPIO port
  //		These bits are written by software to configure the I/O mode.
  //		00: Input mode (reset state)
  //		01: General purpose output mode
  //		10: Alternate function mode
  //		11: Analog mode
  GPIOC->MODER &= ~(GPIO_MODER_MODER5_Msk | GPIO_MODER_MODER4_Msk);
  // Set MODER as alternate function
  GPIOC->MODER |= (0b10 << GPIO_MODER_MODER4_Pos | 0b10 << GPIO_MODER_MODER5_Pos);
  // Alternate function mapping
  GPIOC->AFR[0] |= (GPIO_AF7_USART1 << GPIO_AFRL_AFRL4_Pos | GPIO_AF7_USART1 << GPIO_AFRL_AFRL5_Pos );
  // Configure full speed
  //		These bits are written by software to configure the I/O output speed.
  //		x0: Low speed
  //		01: Medium speed
  //		11: High speed
  GPIOC->OSPEEDR |= ((0b11 << GPIO_OSPEEDER_OSPEEDR4_Pos) | (0b11 << GPIO_OSPEEDER_OSPEEDR5_Pos));
  // Configure UART peripheral
  USART1->CR1 &= ~(USART_CR1_M0_Msk); 			// Reset state -> Start bit, 8 data bits, n stop bits
  USART1->CR1 &= ~(USART_CR1_OVER8_Msk);		// Reset state -> Oversampling by 16
  uint32_t uartdiv = 72000000/9600;				// calculate baund rate
  USART1->BRR = (((uartdiv/16) << USART_BRR_DIV_MANTISSA_Pos) | ((uartdiv%16) << USART_BRR_DIV_FRACTION_Pos));	// calculate BRR
  USART1->CR2 &= ~(USART_CR2_STOP_Msk);			// Reset state -> 1 stop bit
  USART1->CR1 &= ~(USART_CR1_PCE_Msk);			// Reset state -> Parity control disabled
  // Enable RXNEIE interrupt
  //		This bit is set and cleared by software.
  //		0: Interrupt is inhibited
  //		1: A USART interrupt is generated whenever ORE=1 or RXNE=1 in the USART_ISR register
  USART1->CR1 |= (USART_CR1_RXNEIE);			// Enable RXNEIE interrupt
  // ENABLE TXEIE interrupt
  //USART1->CR1 |= (USART_CR1_TXEIE);
  // USART clear TC transfer complete flag
  //USART1->ICR &= ~(USART_ICR_TCCF);
  // Enable USART1
  //		0: USART prescaler and outputs disabled, low-power mode
  //		1: USART enabled
  USART1->CR1 |= (USART_CR1_UE);
  // Enable Transmitter
  //		This bit enables the transmitter. It is set and cleared by software.
  //		0: Transmitter is disabled
  //		1: Transmitter is enabled
  USART1->CR1 |= (USART_CR1_TE);
  // Enable receiving
  //		This bit enables the receiver. It is set and cleared by software.
  //		0: Receiver is disabled
  //		1: Receiver is enabled and begins searching for a start bit
  USART1->CR1 |= (USART_CR1_RE);
  // NVIC set priority
  NVIC_SetPriority(USART1_IRQn, 6);
  // NVIC Enable
  NVIC_EnableIRQ(USART1_IRQn);

  // DMA1 Clock enable
  RCC->AHBENR |= (RCC_AHBENR_DMA1EN);
  // DMA peripheral USART TX buffer address
  DMA1_Channel4->CPAR = (uint32_t)&USART1->TDR;
  // Transfer direction mem to peri | Medium Priority | Memory increment enable
  DMA1_Channel4->CCR |= (DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_PL_0);
  // DMA start channel
  DMA1_Channel4->CCR |= (DMA_CCR_EN);
  // NVIC set priority
  NVIC_SetPriority(DMA1_Channel4_IRQn, 0);
  // NVIC Enable
  NVIC_EnableIRQ(DMA1_Channel4_IRQn);

  //USB_UART_Connect(&huart1);

  HAL_GPIO_WritePin(VOUT_EN_GPIO_Port, VOUT_EN_Pin, 0);
  HAL_GPIO_WritePin(V_SEL_GPIO_Port, V_SEL_Pin, 1);
  HAL_GPIO_WritePin(LED_5V_GPIO_Port, LED_5V_Pin, 0);

  char buf[3] = {'O','N','\n'};
  //HAL_UART_Receive_IT(&huart1, &UART_Received, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /*if(UARTDMA_IsDataReady(&huartdma))
	  {
		  UARTDMA_GetLineFromBuffer(&huartdma, ParseBuffer);
		  if(strcmp(ParseBuffer, "ON") == 0)
		  {
			  HAL_GPIO_TogglePin(LED_5V_GPIO_Port, LED_5V_Pin);
		  }
		  else if(strcmp(ParseBuffer, "OFF")  == 0)
		  {
			  HAL_GPIO_TogglePin(LED_5V_GPIO_Port, LED_5V_Pin);
		  }
		  else {
			  HAL_GPIO_WritePin(V_SEL_GPIO_Port, V_SEL_Pin, 0);
		  }
	  }*/
	  HAL_Delay(1000);
	  //uint32_t isr = USART1->ISR;
	  //uint32_t cr1 = USART1->CR1;
	  //USART1->TDR = 'A';

	    // DMA flash TX buffer address
		DMA1_Channel4->CMAR = (uint32_t)&buf[0];
		// DMA flash TX buffer size
		DMA1_Channel4->CNDTR = 1;
		// USART clear TC transfer complete flag
		USART1->ICR &= ~(USART_ICR_TCCF);
		// Enable DMA transmitter
		USART1->CR3 |= (USART_CR3_DMAT);


	  if (UART_RecFlag){
		  // After reciving data from UART send it throught USB
		  UART_RecFlag = 0;
		  CDC_Transmit_FS(&UART_Received, 1);
	  }
	  //HAL_UART_Transmit(&huart1, buf, 3, 1000);
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
  HAL_GPIO_WritePin(GPIOE, SEG_D_Pin|SEG_B_Pin|SEG_4_Pin|SEG_A_Pin
                          |SEG_C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(V_SEL_GPIO_Port, V_SEL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_5V_Pin|VOUT_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SEG_E_Pin|SEG_G_Pin|SEG_F_Pin|SEG_H_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USB_TX_DIODE_Pin|USB_RX_DIODE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SEG_D_Pin SEG_B_Pin SEG_4_Pin SEG_A_Pin
                           SEG_C_Pin */
  GPIO_InitStruct.Pin = SEG_D_Pin|SEG_B_Pin|SEG_4_Pin|SEG_A_Pin
                          |SEG_C_Pin;
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

  /*Configure GPIO pins : LED_5V_Pin VOUT_EN_Pin */
  GPIO_InitStruct.Pin = LED_5V_Pin|VOUT_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG_E_Pin SEG_G_Pin SEG_F_Pin SEG_H_Pin */
  GPIO_InitStruct.Pin = SEG_E_Pin|SEG_G_Pin|SEG_F_Pin|SEG_H_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DET_Pin */
  GPIO_InitStruct.Pin = DET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_TX_DIODE_Pin USB_RX_DIODE_Pin */
  GPIO_InitStruct.Pin = USB_TX_DIODE_Pin|USB_RX_DIODE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
