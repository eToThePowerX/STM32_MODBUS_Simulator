/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32_tm1637.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#define slave_id 0x01

enum res{VOLTAGE, CURRENT, REACTIVE_POWER, POWER_FACTOR, DO};
enum device_type{Energy_Meter, Dissolve_Oxygen, Temp_Humi, Pressure_Meter, Oxygen_Sensor, pH, Flow_Meter};

uint16_t adc_val(void);

void send(enum res r, uint16_t nBytes);
uint8_t *hex_array(float f);
uint16_t XOR(uint16_t constant, uint16_t datum);
void crc(uint8_t len, uint16_t cvalue[]);

void getParameterOf(enum device_type dt);

float VP[] = {0.0, 0.0, 0.0};
float AMP[] = {0.0, 0.0, 0.0};
float RAP[] = {0.0, 0.0, 0.0};
float PF[] = {0.0, 0.0, 0.0, 0.0};

float DO_mgL, DO_Water_Temp, DO_percentage;

uint8_t upperCRC, lowerCRC;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define UARTBuf_SIZE 250
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
uint8_t uart_response[UARTBuf_SIZE] = {0x00};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart->Instance == USART2)
	{
	    if(uart_response[0] == slave_id && uart_response[1] == 0x03)
	    {
	        uint16_t sAddress = (uart_response[2] << 8) + uart_response[3];
	        uint16_t nBytes = (uart_response[4] << 8) + uart_response[5];
	        uint16_t vBytes[6];
	        for(uint8_t i=0;i<6;i++)
	        {
	            vBytes[i] = uart_response[i];
	        }
	        crc(6,vBytes);
	        if(lowerCRC == uart_response[6] && upperCRC == uart_response[7])
	        {
	            if(sAddress == 0x0100)
	            {
	                send(VOLTAGE, nBytes);
	            }
	            else if(sAddress == 0x0120)
	            {
	                send(CURRENT, nBytes);
	            }
	            else if(sAddress == 0x014E)
	            {
	                send(REACTIVE_POWER, nBytes);
	            }
	            else if(sAddress == 0x0132)
	            {
	                send(POWER_FACTOR, nBytes);
	            }

	            else if(sAddress == 0x2100 || sAddress == 0x2000)
	            {
	            	send(DO, nBytes);
	            }
	        }
	    }

		for(uint8_t i=0;i<UARTBuf_SIZE;i++)
		{
			uart_response[i] = 0x00;
		}

		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart_response, UARTBuf_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
	}
}
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart_response, UARTBuf_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  tm1637Init();
  tm1637SetBrightness(8);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  /*getParameterOf(Energy_Meter);
	  for(uint8_t i=0;i<3;i++)
	  {
		  tm1637DisplayDecimal(VP[i],0);
		  HAL_Delay(1000);
		  tm1637DisplayDecimal(AMP[i]*100,2);
		  HAL_Delay(1000);
		  tm1637DisplayDecimal(RAP[i],0);
		  HAL_Delay(1000);
		  tm1637DisplayDecimal(PF[i]*100,2);
		  HAL_Delay(1000);
	  }
	  tm1637DisplayDecimal(PF[3]*100,2);
	  HAL_Delay(5000);*/

	  getParameterOf(Dissolve_Oxygen);
	  tm1637DisplayDecimal((DO_mgL*100), 2);
	  HAL_Delay(1000);
	  tm1637DisplayDecimal((DO_Water_Temp*100), 2);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|CLK_Pin|DIO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA5 CLK_Pin DIO_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_5|CLK_Pin|DIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void send(enum res r, uint16_t nBytes)
{
    uint8_t *arr;

    uint8_t sz = 5 + (nBytes * 2);
    uint8_t slave_send[sz];
    slave_send[0] = slave_id;
    slave_send[1] = 0x03;
    slave_send[2] = nBytes * 2;

    uint8_t k = 3;
    for(uint8_t j=0;j<(nBytes/2);j++)
    {
        if(r == VOLTAGE)
        {
            if(j>2)
            {
                arr = hex_array(0.0);
            }
            else
            {
                arr = hex_array(VP[j]);
            }
        }
        else if(r == CURRENT)
        {
            if(j>2)
            {
                arr = hex_array(0.0);
            }
            else
            {
                arr = hex_array(AMP[j]);
            }
        }
        else if(r == REACTIVE_POWER)
        {
            if(j>2)
            {
                arr = hex_array(0.0);
            }
            else
            {
                arr = hex_array(RAP[j]);
            }
        }
        else if(r == POWER_FACTOR)
        {
            if(j>3)
            {
                arr = hex_array(0.0);
            }
            else
            {
                arr = hex_array(PF[j]);
            }
        }
        else if(r == DO)
        {
            if(j>3)
            {
                arr = hex_array(0.0);
            }
            else
            {
            	if(nBytes == 0x0002)
            	{
                	uint32_t num = *((uint32_t*)&DO_mgL);
                	arr[3] = (num & 0xFF000000) >> 24;
                	arr[2] = (num & 0x00FF0000) >> 16;
                	arr[1] = (num & 0x0000FF00) >> 8;
                	arr[0] = (num & 0x000000FF);
            	}
            	else
            	{
                	uint32_t num = *((uint32_t*)&DO_Water_Temp);
                	arr[3] = (num & 0xFF000000) >> 24;
                	arr[2] = (num & 0x00FF0000) >> 16;
                	arr[1] = (num & 0x0000FF00) >> 8;
                	arr[0] = (num & 0x000000FF);
            	}
            }
        }

        for(uint8_t i=0;i<4;i++)
        {
            slave_send[i+k] = arr[i];
        }
        k += 4;
    }

    uint16_t vBytes[(nBytes * 2) + 3];
    for(uint8_t i=0;i<((nBytes * 2)+3);i++)
    {
        vBytes[i] = slave_send[i];
    }

    if(nBytes > 6)
    {
        while(k < (nBytes+3))
        {
            slave_send[k++] = 0x00;
        }
    }
    crc(((nBytes*2)+3),vBytes);
    slave_send[k++] = lowerCRC;
    slave_send[k++] = upperCRC;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart2, slave_send, sizeof(slave_send), 500);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
}

uint8_t *hex_array(float f)
{
    static uint8_t reply[4];
    uint32_t num = *((uint32_t*)&f);
    //printf("0x%X\n",num);
    reply[0] = (num & 0x0000FF00) >> 8;
    reply[1] = (num & 0x000000FF);
    reply[2] = (num & 0xFF000000) >> 24;
    reply[3] = (num & 0x00FF0000) >> 16;
    return reply;
}

uint16_t XOR(uint16_t constant, uint16_t datum)
{
  uint8_t i;
  uint16_t polynomial = 0xA001;
  datum = constant ^ datum;
  for(i=0;i<8;i++)
  {
    if((datum & (0x0001)) == 0)
    {
      datum = datum >> 1;
      datum = datum & (0x7FFF);
    }
    else
    {
      datum = datum >> 1;
      datum = datum & (0x7FFF);
      datum = datum ^ polynomial;
    }
  }
  return datum;
}

void crc(uint8_t len, uint16_t cvalue[])
{
	uint16_t finalCRC, j;

  for(j=0;j<len;j++)
  {
    if(j==0)
    {
      cvalue[j] = XOR(0xFFFF,cvalue[j]);
    }
    else
    {
      cvalue[j] = XOR(cvalue[j-1],cvalue[j]);
      finalCRC = cvalue[j];
    }
    if(j==len-1)
    {
      upperCRC = (finalCRC & (0xFF00)) >> 8;
      lowerCRC = finalCRC & (0x00FF);
    }
  }
}

uint16_t adc_val(void)
{
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	uint16_t readValue = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	HAL_Delay(50);

	return readValue;
}

void getParameterOf(enum device_type dt)
{
	if(dt == Energy_Meter)
	{
		for(uint8_t i=0;i<3;i++)
		{
			VP[i] = 180.0 + ((100.0/4095.0) * adc_val());
			AMP[i] = 1.0 + ((1.0/4095.0) * adc_val());
			PF[i] = 0.8 + ((0.2/4095.0) * adc_val());
			RAP[i] = VP[i] * AMP[i] * PF[i]; //Although this is active power.
		}
		PF[3] = (PF[0] + PF[1] + PF[2]) / 3.0;
	}
	else if(dt == Dissolve_Oxygen)
	{
		DO_mgL = ((20.0/4095.0) * adc_val());
		DO_Water_Temp = 5.0 + ((45.0/4095.0) * adc_val());
		DO_percentage = ((100.0/4095.0) * adc_val());
	}
	else if(dt == Temp_Humi)
	{
		;
	}
	else if(dt == Pressure_Meter)
	{
		;
	}
	else if(dt == Oxygen_Sensor)
	{
		;
	}
	else if(dt == pH)
	{
		;
	}
	else if(dt == Flow_Meter)
	{
		;
	}
}
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
