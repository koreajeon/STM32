/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "CLCD.h"
#include "7seg.h"
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
uint8_t rx3_data;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
int _write(int file, char* p, int len)
{
	HAL_UART_Transmit(&huart3, p, len, 10);
	return len;
}
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
	volatile uint16_t adcval[4];
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_TIM7_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM10_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_I2C1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);

	HAL_UART_Receive_IT(&huart3, &rx3_data, 1);
	HAL_TIM_Base_Start_IT(&htim7);

	_7SEG_GPIO_Init();

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

	HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1); //servo

	//HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //buzzer

	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1); //DC
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4); //DC


	//HAL_ADC_Start_DMA(&hadc1, &adcval[0], 4); //DMA

	HAL_DAC_Start(&hdac, DAC_CHANNEL_1); //DAC

	TIM10->CCR1 = 1500;

	//부저 주파수 상승
	//TIM2->ARR = 500;
	//TIM2->CCR1 = TIM2->ARR / 2;

	// 시계방향 회전(DC)
	TIM5->CCR1 = 0;
	TIM5->CCR4 = 5000; // 0 ~ 10000 값에 비례하게 회전 속도제어
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint8_t a = 0;
	float f = 1.234;
	uint8_t str[20];
	uint16_t ccr = 0;
	uint16_t arr = 1000;
	uint8_t ud_flag = 0;
	uint16_t dacval = 0;
	uint8_t eeprom[10] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99}; //v/

	HAL_I2C_Mem_Write(&hi2c1, 0xA0, 0x00, I2C_MEMADD_SIZE_8BIT, &eeprom[0], 10, 10); //v/

	HAL_Delay(3); //v/

	for(int i = 0; i<10; i++) eeprom[i] = 0x00; //v/

	HAL_I2C_Mem_Read(&hi2c1, 0xA0, 0x00, I2C_MEMADD_SIZE_8BIT, &eeprom[0], 10, 10); //v/

	sprintf(str, "%02x %02x %02x %02x %02x", eepron[0], eepron[1], eepron[2], eepron[3], eepron[4]); //v/
	CLCD_Puts(0, 0, str); //v/
	sprintf(str, "%02x %02x %02x %02x %02x", eepron[5], eepron[6], eepron[7], eepron[8], eepron[9]); //v/
	CLCD_pits(0, 1, str); //v/

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* TIM7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM7_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART3)
	{
		HAL_UART_Receive_IT(&huart3, &rx3_data, 1);
		HAL_UART_Transmit(&huart3, &rx3_data, 1, 10);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static unsigned char cnt = 0;
	if(htim->Instance == TIM7)
	{

		cnt++;
	}
}

//외부 인터럽트가 걸리면 이 함수 호출
//함수 안에서 어떤 스위치가 눌렸는지 구분
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_3)
	{
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
	}
	if(GPIO_Pin == GPIO_PIN_15)
	{
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);

	}
	if(GPIO_Pin == GPIO_PIN_4)
	{
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);

	}
	if(GPIO_Pin == GPIO_PIN_10)
	{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
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
