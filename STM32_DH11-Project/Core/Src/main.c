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
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ESPDataLogger.h";
#include "usbd_cdc_if.h"
#include "string.h"
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
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
void delay(uint16_t time){

	__HAL_TIM_SET_COUNTER(&htim6,0);

	while(__HAL_TIM_GET_COUNTER(&htim6)< time);

}


void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){


	GPIO_InitTypeDef DHT11_DATA={0};

	DHT11_DATA.Pin=GPIO_Pin;
	DHT11_DATA.Mode=GPIO_MODE_OUTPUT_PP;
	DHT11_DATA.Pull=GPIO_NOPULL;
	DHT11_DATA.Speed=GPIO_SPEED_FREQ_LOW;

	HAL_GPIO_Init(GPIOx,&DHT11_DATA);

}

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){


	GPIO_InitTypeDef DHT11_DATA={0};

	DHT11_DATA.Pin=GPIO_Pin;
	DHT11_DATA.Mode=GPIO_MODE_INPUT;
	DHT11_DATA.Pull=GPIO_NOPULL;
	DHT11_DATA.Speed=GPIO_SPEED_FREQ_LOW;

	HAL_GPIO_Init(GPIOx,&DHT11_DATA);

}

#define DHT11_PORT GPIOA
#define DHT11_PIN GPIO_PIN_3



uint8_t durum=0,Humidity=0,Temperature=0;
uint16_t tempVal=0,humVal=0;
uint8_t dhtVal[2];
uint8_t mData[40];
uint16_t mTime1 = 0, mTime2 = 0;
uint16_t mbit = 0;
uint8_t  parityVal = 0, genParity = 0;

uint8_t DHT11_Read (void){

  for(int a=0;a<40;a++) mData[a]=0;
   mTime1 = 0, mTime2 = 0, durum=0, tempVal=0, humVal=0, parityVal = 0, genParity = 0,  mbit = 0;

     Set_Pin_Output(DHT11_PORT,DHT11_PIN);
	 HAL_GPIO_WritePin(DHT11_PORT,DHT11_PIN,GPIO_PIN_RESET);
    delay(18000);
   	Set_Pin_Input(DHT11_PORT,DHT11_PIN);

	 __HAL_TIM_SET_COUNTER(&htim6, 0);				//set timer counter to zero
	while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET) if((uint16_t)__HAL_TIM_GET_COUNTER(&htim6) > 500) return 0;

	__HAL_TIM_SET_COUNTER(&htim6, 0);
	while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_RESET) if((uint16_t)__HAL_TIM_GET_COUNTER(&htim6) > 500) return 0;
	mTime1 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim6);

	__HAL_TIM_SET_COUNTER(&htim6, 0);
	while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET) if((uint16_t)__HAL_TIM_GET_COUNTER(&htim6) > 500) return 0;
    mTime2 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim6);


	if(mTime1 < 75 && mTime1 > 85 && mTime2 < 75 && mTime2 > 85)
	{

		return 0;
	}




	for(int j = 0; j < 40; j++)
	{
		__HAL_TIM_SET_COUNTER(&htim6, 0);
		while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_RESET) if((uint16_t)__HAL_TIM_GET_COUNTER(&htim6) > 500) return 0;
		__HAL_TIM_SET_COUNTER(&htim6, 0);
		while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET) if((uint16_t)__HAL_TIM_GET_COUNTER(&htim6) > 500) return 0;
		mTime1 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim6);

		//check pass time in high state
		//if pass time 25uS set as LOW
		if(mTime1 > 20 && mTime1 < 30)
		{
			mbit = 0;
		}
		else if(mTime1 > 60 && mTime1 < 80) //if pass time 70 uS set as HIGH
		{
			 mbit = 1;
		}

		//set i th data in data buffer
		mData[j] = mbit;

	}

	//get hum value from data buffer
	for(int i = 0; i < 8; i++)
	{
		humVal += mData[i];
		humVal = humVal << 1;
	}

	//get temp value from data buffer
	for(int i = 16; i < 24; i++)
	{
		tempVal += mData[i];
		tempVal = tempVal << 1;
	}

	//get parity value from data buffer
	for(int i = 32; i < 40; i++)
	{
		parityVal += mData[i];
		parityVal = parityVal << 1;
	}

	parityVal = parityVal >> 1;
	humVal = humVal >> 1;
	tempVal = tempVal >> 1;

	genParity = humVal + tempVal;


	dhtVal[0]= tempVal;
	dhtVal[1] = humVal;

	return 1;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t dht_Val[2];
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
  MX_TIM6_Init();
  MX_USART6_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim6);
  ESP_Init("TTNET_AirTies_Air5650_8EG6","060956caak.");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  durum=DHT11_Read();
		  	      if (durum ==1)	{Temperature=tempVal; Humidity=humVal;}
		  	      HAL_Delay(1000);
		  	     dht_Val[0]=tempVal;
		  	   dht_Val[1]=humVal;


		  	      ESP_Send_Multi("BEOXAEQ0EAYGUXQ9",2,dht_Val);
		  	     HAL_Delay (15000);

		  	   char usbdenGonderilecekData[50];
		  	  char usbdenGonderilecekData_2[50];
		  	  	  sprintf(usbdenGonderilecekData, "Isi degeri: %d C \n",Temperature);
		  	  	 sprintf(usbdenGonderilecekData_2, "Nem degeri: %d g/m3 \n",Humidity);

		  	  	 // uint8_t buf[5] = "Hello";
		  	  	  CDC_Transmit_FS(usbdenGonderilecekData, 25);
		  	  	  HAL_Delay(100);
		  	  	 CDC_Transmit_FS(usbdenGonderilecekData_2, 25);
HAL_Delay(1000);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 84-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 0xffff-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
