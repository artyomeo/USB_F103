/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t TxBuffer[1024] = {0};
uint16_t TxBufferLen = sizeof(TxBuffer);

uint8_t RxBuffer[1024] = {0};
uint8_t RxBuffertemp[1024] = {0};
uint16_t RxBufferLen = sizeof(RxBuffer);
uint16_t RxMessageLen = 0;
uint16_t RxMessageLentemp = 0;

struct UART_settings
{
	uint32_t baudrate;
	uint8_t stopbits;
	uint8_t parity;
	uint8_t datalen;
};
struct UART_settings uart_settings_external = {.baudrate = 9600, .datalen = 8, .parity = 0, .stopbits = 0};

UART_InitTypeDef uart_settings_internal = {.BaudRate = 9600, .WordLength = UART_WORDLENGTH_8B, .StopBits = UART_STOPBITS_1, \
										   .Parity = UART_PARITY_NONE, .Mode = UART_MODE_TX_RX, .HwFlowCtl = UART_HWCONTROL_NONE, \
										   .OverSampling = UART_OVERSAMPLING_16};

extern uint8_t flag_rx_settings;
extern uint16_t len_rx_settings;

extern uint8_t buf_rx_message[1024];
extern uint16_t len_rx_message;
extern uint8_t flag_rx_message;

extern uint8_t flag_rx_controlline;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //CDC_Transmit_FS(TxBuffer, TxBufferLen);
      //HAL_Delay(100);

	  if (flag_rx_settings)
	  {
		  memset(TxBuffer, '\0', TxBufferLen);
		  sprintf((char *)TxBuffer, "Baudrate: %ld; stopbits: %d; parity: %d; datalen: %d; settingslen: %d\r\n",\
				  uart_settings_external.baudrate, uart_settings_external.stopbits, uart_settings_external.parity, uart_settings_external.datalen, len_rx_settings);
		  CDC_Transmit_FS(TxBuffer, strlen((char *)TxBuffer));
		  len_rx_settings = 0;
		  flag_rx_settings = 0;

		  MX_USART2_UART_Init();
	  }

	  if (flag_rx_message)
	  {
		  CDC_Transmit_FS(buf_rx_message, len_rx_message);
		  HAL_UART_Transmit(&huart2, buf_rx_message, len_rx_message, 100);
		  len_rx_message = 0;
		  flag_rx_message = 0;
	  }

	  if (RxMessageLen)
	  {
		  RxMessageLentemp = RxMessageLen;
		  memcpy (RxBuffertemp, RxBuffer, RxMessageLentemp);
		  UART_Start_Receive_IT (&huart2, RxBuffer, RxBufferLen);
		  RxMessageLen = 0;

		  CDC_Transmit_FS(RxBuffertemp, RxMessageLentemp);
	  }

	  HAL_Delay(100);
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
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */
	if (uart_settings_external.baudrate)
		uart_settings_internal.BaudRate = uart_settings_external.baudrate;

	switch (uart_settings_external.datalen)
	{
		case 9:
			uart_settings_internal.WordLength = UART_WORDLENGTH_9B;
		break;
		default:
			uart_settings_internal.WordLength = UART_WORDLENGTH_8B;
		break;
	}

	switch (uart_settings_external.parity)
	{
		case 1:
			uart_settings_internal.Parity = UART_PARITY_ODD;
		break;
		case 2:
			uart_settings_internal.Parity = UART_PARITY_EVEN;
		break;
		default:
			uart_settings_internal.Parity = UART_PARITY_NONE;
		break;
	}

	switch (uart_settings_external.stopbits)
	{
		case 2:
			uart_settings_internal.StopBits = UART_STOPBITS_2;
		break;
		default:
			uart_settings_internal.StopBits = UART_STOPBITS_1;
		break;
	}

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
	if(0)
	{
  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
	}
	else
	{
		  huart2.Instance = USART2;
		  huart2.Init = uart_settings_internal;
		  if (HAL_UART_Init(&huart2) != HAL_OK)
		  {
		    Error_Handler();
		  }
		  UART_Start_Receive_IT (&huart2, RxBuffer, RxBufferLen);
		  //huart2.pRxBuffPtr = RxBuffer;
	}
  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
