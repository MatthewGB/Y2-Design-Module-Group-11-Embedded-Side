/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>  // Required for strlen() function
#include <stdio.h>   // Required for sprintf() function
#include <stdlib.h>  // Required for abs() function
#include <math.h>
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
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define MAX_DAC_NUMBER 4095
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void serialOut(UART_HandleTypeDef *huart, char _out[], uint32_t len){
	HAL_UART_Transmit(huart, (uint8_t *) _out, len, 100);
}

void printStr(char str[])
{
	serialOut(&huart2, str, strlen(str));
}

void printChar(char chr)
{
	char str1[2] = {chr , '\0'};
	char str2[5] = "";
	strcpy(str2,str1);
	serialOut(&huart2, str2, strlen(str2));
}

void printInt(int i)
{
	char str[5];
	itoa(i, str, 10);
	serialOut(&huart2, str, strlen(str));
}

void printIntLn(int i)
{
	char str[5];
	itoa(i, str, 10);
	serialOut(&huart2, str, strlen(str));
	printStr("\r\n");
}

/**
 * Print string and go to next line (currently broken)
 */
void printStrLn(char str[])
{
	strcat(str, "\r\n");
	serialOut(&huart2, str, strlen(str));
}

void printWaveform(short data[], int size)
{
	printStr("<WF>");
	for(int i = 0; i<size; i++)
	{
		printStr("|");
		printInt(data[i]);
	}
	printStr("</WF>");
}


void readSerial(char* outputString, int readsize)
{
	char rxedString[readsize+1];
	for(int i = 0; i<readsize; i++)
	{
		rxedString[i] = '#';
	}
	char rxedChar[1] = "#";
	int charnum = 0;
	while (1)
	{
		  HAL_UART_Receive(&huart2, (uint8_t *)rxedChar, 1, 100);

		  if (rxedChar[0] == '\n' || rxedChar[0] == '\r') {
			  break;
		  }

		  if(rxedChar[0] != '#')
		  {
			  rxedString[charnum] = rxedChar[0];
			  charnum += 1;
			  /*
			  for(int i = 0; i<readsize; i++)
			  {
				  printChar(rxedString[i]);
			  }*/
			  rxedChar[0] = '#';
		  }

		  if(rxedString[readsize-1] != '#')
		  {
			  break; //String is full
		  }
	}

	int truesize = 0;
	for(int i = 0; i<readsize; i++)
	{
		if(rxedString[i] != '#')
		{
			truesize += 1;
		}
	}
	rxedString[truesize] = 0; //Terminates string correctly

	strcpy(outputString, rxedString);
}

/**
 * Read one value from the ADC
 */
short readADC()
{
	HAL_ADC_Start(&hadc1);
	//HAL_ADC_PollForConversion(&hadc1, 10);
	uint32_t value = HAL_ADC_GetValue(&hadc1);
	return value;
}

/**
 * Get the set amount of samples in the timeframe, and store in data
 */
int getWaveform(short* data, int samples, int timeframe)
{
	//219,780 samples per second
	//printInt(1);
	//printInt((samples/219780.0)*1000.0);

	if((samples/219780.0)*1000.0 > timeframe) //If data is sampled often enough, delay isn't needed (eg <8.7ms at 1920 samples)
	{
		int start = HAL_GetTick();
		for(int i = 0; i < samples; i++)
		{
			data[i] = readADC();
		}
		printStr("Acquisition time (ms): ");
		printInt(HAL_GetTick()-start);
	}
	//printInt(HAL_GetTick()-start);
}

/**
 * Allows for more samples to be taken and then averaged
 */
void compressWaveform(short* data, int samples, int resolution_x, int resolution_y)
{
	int samples_per_pixel = samples/resolution_x;
	int current_pixel = 0;
	short newdata[resolution_x];
	newdata[current_pixel] += data[0];
	for(int i = 1; i<samples;)
	{
		while(i % samples_per_pixel != 0)
		{
			newdata[current_pixel] += data[i];
			i++;
		}
		//newdata[current_pixel];
		current_pixel++;
	}
}
/*
void miliSleep(int sleeptime, int sleepcal)
{
	for(int i = 0; i<3786*sleeptime; i++)
	{
		int x = pow(2,12);
	}
}

void microSleep(int sleeptime, int sleepcal)
{
	for(int i = 0; i<3786*sleeptime; i++)
	{
		int x = pow(2,12);
	}
}*/
/*
int sleepCalibration()
{
	int start = HAL_GetTick();
	for(int i = 0; i<500000; i++)
	{
		int x = pow(2,12);
	}
	int end = (HAL_GetTick()-start);
	return (500000/end)*1.16;
}*/
/* USER CODE END 0 */


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  int resolution_x = 1920;
  int resolution_y = 1080;

  int RAM_size = 80000;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  // One source on line says you have to manually enable the system clock to the DAC.
  // It doesn't seem to matter when I try it, but if required, this is the code:
  // RCC->APB1ENR |= RCC_APB1ENR_DAC1EN;

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  /* USER CODE BEGIN 2 */
  printStr("Starting up...");

  int max_samples = (RAM_size / 2) * 0.05 ; //80kB = 40k 16 bit shorts
  max_samples = (max_samples / resolution_x) * resolution_x; //Helps with compression
  printStr("Max samples set to: "); printInt(max_samples); printStr("\n\r");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //printInt(readADC());
  //int sleep_cal = sleepCalibration();
  //printInt(sleep_cal);
  printStr("Ready\n\r");
  short data[max_samples];
  getWaveform(data, max_samples, 8);
  printWaveform(data, max_samples);



  /*
  int sleep_cal = 1;
  int start = HAL_GetTick();
  sleep(1000, sleep_cal);
  printIntLn(HAL_GetTick()-start);
  start = HAL_GetTick();

  sleep(100, sleep_cal);
  printIntLn(HAL_GetTick()-start);
  start = HAL_GetTick();
  sleep(10, sleep_cal);
  printIntLn(HAL_GetTick()-start);
  start = HAL_GetTick();
  sleep(1, sleep_cal);
  printIntLn(HAL_GetTick()-start);
  start = HAL_GetTick();
  sleep(2000, sleep_cal);
  printIntLn(HAL_GetTick()-start);*/

  //short data[20000];
  //getWaveform(data, 20000, 1);
  //printStr("got data");
  //printWaveform(data, 50);

  //serialOut(&huart2, hello, strlen(hello));

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
