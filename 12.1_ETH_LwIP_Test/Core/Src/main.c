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
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "lwip.h"
#include "sdio.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CLCD.h"
#include "7seg.h"
#include <math.h>
#include "VS1003.h"
#include "MP3Sample.h"
#include "udp_echoserver.h"
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
uint8_t dd = 0;

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

  /* USER CODE END Init */

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
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_LWIP_Init();

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

	//HAL_DAC_Start(&hdac, DAC_CHANNEL_1); //DAC

	TIM10->CCR1 = 1500;

	//ï¿???? ì£¼íŒŒ?ï¿½ï¿½ ?ï¿½ï¿½?ï¿½ï¿½
	//TIM2->ARR = 500;
	//TIM2->CCR1 = TIM2->ARR / 2;

	// ?ï¿½ï¿½ê³„ë°©?ï¿½ï¿½ ?ï¿½ï¿½?ï¿½ï¿½(DC)
	TIM5->CCR1 = 0;
	TIM5->CCR4 = 5000; // 0 ~ 10000 ê°’ì— ë¹„ï¿½??ï¿½ï¿½ï¿?? ?ï¿½ï¿½?ï¿½ï¿½ ?ï¿½ï¿½?ï¿½ï¿½?ï¿½ï¿½?ï¿½ï¿½

	VS1003_Init();
	VS1003_SoftReset();

  /* tcp echo server Init */
	udp_echoserver_init();

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
	uint8_t eeprom[10];// = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99};

	//HAL_I2C_Mem_Write(&hi2c1, 0xA0, 0x00, I2C_MEMADD_SIZE_8BIT, &eeprom[0], 10, 10);

	HAL_Delay(3);

	HAL_I2C_Mem_Read(&hi2c1, 0xA0, 0x00, I2C_MEMADD_SIZE_8BIT, &eeprom[0], 10, 10);

	sprintf(str, "%02x %02x %02x %02x %02x", eeprom[0], eeprom[1], eeprom[2], eeprom[3], eeprom[4]);
	CLCD_Puts(0, 0, str);
	sprintf(str, "%02x %02x %02x %02x %02x", eeprom[5], eeprom[6], eeprom[7], eeprom[8], eeprom[9]);
	CLCD_Puts(0, 1, str);
	int i =0;

	BYTE buf[32] = "Hello world"; // file copy buffer

	uint32_t bw, br;
	/************* 1. FSï¿???ï¿½ï¿½ ì´ˆê¸°?ï¿½ï¿½ "0:/" ?ï¿½ï¿½?ï¿½ï¿½?ï¿½ï¿½ï¿?? ?ï¿½ï¿½?ï¿½ï¿½ ***********/
	if((retSD = f_mount(&SDFatFS, &SDPath[0], 1)) == FR_OK)
	{
		sprintf(str, "f_mount OK %d", retSD);
		CLCD_Puts(0, 0, str);
	}
	else
	{
		sprintf(str, "f_mount failed %d", retSD);
		CLCD_Puts(0, 0, str);
	}

	/********************************************************/

//	/*********** 2. ?ï¿½ï¿½?ï¿½ï¿½ ?ï¿½ï¿½?ï¿½ï¿½ ï¿?? ?ï¿½ï¿½ï¿?? ***********/
//	if((retSD = f_open(&SDFile, "0:/M-HIVE.txt", FA_CREATE_NEW | FA_WRITE)) == FR_OK)
//	{
//		sprintf(buf, "Hello world");
//		f_write(&SDFile, buf, sizeof(buf), &bw);
//
//		sprintf(str, "%d bytes", bw);
//		CLCD_Puts(0, 0, str);
//
//		f_close(&SDFile);
//	}
//	else
//	{
//		sprintf(str, "error %d\n", retSD);
//		CLCD_Puts(0, 0, str);
//	}
//	/*****************************************************/

//	/********** 3. ?ï¿½ï¿½?ï¿½ï¿½ ?ï¿½ï¿½?ï¿½ï¿½?ï¿½ï¿½ ?ï¿½ï¿½ï¿??  ************/
//	if((retSD = f_open(&SDFile, "0:/ChrisP.txt", FA_OPEN_EXISTING | FA_READ)) == FR_OK)
//	{
//		CLCD_Clear();
//
//		f_read(&SDFile, buf, 32, &br);
//		sprintf(str, "%s", buf);
//		CLCD_Puts(0, 0, str);
//
//		sprintf(str, "%d bytes", br);
//		CLCD_Puts(0, 1, str);
//	}
//	else
//	{
//		sprintf(str, "open error %d\n", retSD);
//		CLCD_Puts(0, 0, str);
//	}
//	/*****************************************************/

	/**************** ?ï¿½ï¿½?ï¿½ï¿½?ï¿½ï¿½ ?ï¿½ï¿½?ï¿½ï¿½ ?ï¿½ï¿½ï¿?? ?ï¿½ï¿½ï¿?? *******************/
	DIR dir;
	FILINFO filinfo;
	uint16_t filecnt = 0;
	if((retSD = f_opendir(&dir, "0:/")) == FR_OK) // (0:/) ?ï¿½ï¿½?ï¿½ï¿½ ?ï¿½ï¿½?ï¿½ï¿½?ï¿½ï¿½ dir?ï¿½ï¿½ ï¿?? ?ï¿½ï¿½ë³´ï¿½?? ???ï¿½ï¿½
	{
		while(1) // ?ï¿½ï¿½?ï¿½ï¿½ ï¿???ï¿½ï¿½ ?ï¿½ï¿½ï¿??
		{
			retSD = f_readdir(&dir, &filinfo); //?ï¿½ï¿½?ï¿½ï¿½?ï¿½ï¿½ï¿?? ?ï¿½ï¿½?ï¿½ï¿½ ?ï¿½ï¿½?ï¿½ï¿½?ï¿½ï¿½ï¿?? ?ï¿½ï¿½?ï¿½ï¿½ ?ï¿½ï¿½ï¿??. ?ï¿½ï¿½?ï¿½ï¿½ ?ï¿½ï¿½?ï¿½ï¿½ï¿?? ï¿?? ?ï¿½ï¿½?ï¿½ï¿½?ï¿½ï¿½?ï¿½ï¿½ï¿?? ?ï¿½ï¿½?ï¿½ï¿½ï¿??.
			if(dir.sect == 0) break; //?ï¿½ï¿½?ï¿½ï¿½?ï¿½ï¿½ ?ï¿½ï¿½?ï¿½ï¿½ ?ï¿½ï¿½?ï¿½ï¿½?ï¿½ï¿½ ?ï¿½ï¿½?ï¿½ï¿½ ê²½ìš°

			printf("%s\t0x%x\n", filinfo.fname, filinfo.fattrib);
			filecnt++;
		}
		printf("\nTotal %d file(s)..\n\n", filecnt);
	}
	else
	{
		printf("The file/directory object is invalid! %d\n", retSD);
	}
	/*************************************************/

	/****************** ?ï¿½ï¿½?ï¿½ï¿½ ?ï¿½ï¿½ï¿?? ************************/
	unsigned char filename[20] = "0:/1.mp3";
	uint16_t index = 0; // ?ï¿½ï¿½?ï¿½ï¿½ ?ï¿½ï¿½?ï¿½ï¿½ ???ï¿½ï¿½ ï¿???ï¿½ï¿½ ï¿???ï¿½ï¿½

	if((retSD = f_open(&SDFile, filename, FA_OPEN_EXISTING | FA_READ)) == FR_OK)
	{
		CLCD_Clear();
		sprintf(str, "%s opened", filename);
		CLCD_Puts(0, 0, str);
	}
	else
	{
		sprintf(str, "open error %d\n", retSD);
		CLCD_Puts(0, 0, str);
	}
	/*************************************************/

	uint8_t pp_flag = 0;

  while (1)
  {
	  if(MP3_DREQ == 1)
	  	  {
	  		  if(pp_flag)
	  		  {
	  			  f_read(&SDFile, buf, 32, &br);
	  			  if(br >= 32)
	  			  {
	  				  VS1003_WriteData(&buf[0]);
	  			  }
	  			  else
	  			  {

	  			  }
	  		  }
	  	  }

	  	  if(udp_flag)
	  	  {
	  		  udp_flag = 0;

	  		  switch(udp_data)
	  		  {
	  		  case '1': SW1_flag = 1; break;
	  		  case '2': SW2_flag = 1; break;
	  		  case '3': SW3_flag = 1; break;
	  		  }
	  	  }

	  	  if(SW1_flag)
	  	  {
	  		  SW1_flag = 0;

	  		  index = index == 0 ? 2 : index-1;
	  		  switch(index)
	  		  {
	  		  case 0: strcpy(filename, "0:/1.mp3"); break;
	  		  case 1: strcpy(filename, "0:/2.wav"); break;
	  		  case 2: strcpy(filename, "0:/3.mp3"); break;
	  		  }

	  			f_close(&SDFile);

	  			if((retSD = f_open(&SDFile, filename, FA_OPEN_EXISTING | FA_READ)) == FR_OK)
	  			{
	  				sprintf(str, "%s opened", filename);
	  				CLCD_Puts(0, 0, str);
	  			}
	  			else
	  			{
	  				sprintf(str, "open error %d\n", retSD);
	  				CLCD_Puts(0, 0, str);
	  			}

	  			VS1003_SoftReset();
	  	  }

	  	  if(SW3_flag)
	  	  {
	  		  SW3_flag = 0;

	  		  index = index == 2 ? 0 : index+1;
	  		  switch(index)
	  		  {
	  		  case 0: strcpy(filename, "0:/1.mp3"); break;
	  		  case 1: strcpy(filename, "0:/2.wav"); break;
	  		  case 2: strcpy(filename, "0:/3.mp3"); break;
	  		  }

	  			f_close(&SDFile);

	  			if((retSD = f_open(&SDFile, filename, FA_OPEN_EXISTING | FA_READ)) == FR_OK)
	  			{
	  				sprintf(str, "%s opened", filename);
	  				CLCD_Puts(0, 0, str);
	  			}
	  			else
	  			{
	  				sprintf(str, "open error %d\n", retSD);
	  				CLCD_Puts(0, 0, str);
	  			}

	  			VS1003_SoftReset();
	  	  }

	  	  if(SW2_flag)
	  	  {
	  		  SW2_flag = 0;
	  		  if(pp_flag)
	  		  {
	  			  pp_flag = 0;
	  			  CLCD_Puts(0, 1, "paused          ");
	  		  }
	  		  else
	  		  {
	  			  pp_flag = 1;
	  			  CLCD_Puts(0, 1, "playing         ");

	  		  }
	  	  }


	  	    /* Read a received packet from the Ethernet buffers and send it
	  	       to the lwIP for handling */
	  	    ethernetif_input(&gnetif);

	  	    /* Handle timeouts */
	  	    sys_check_timeouts();
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
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

//?ï¿½ï¿½ï¿?? ?ï¿½ï¿½?ï¿½ï¿½?ï¿½ï¿½?ï¿½ï¿½ï¿?? ê±¸ë¦¬ï¿?? ?ï¿½ï¿½ ?ï¿½ï¿½?ï¿½ï¿½ ?ï¿½ï¿½ï¿??
//?ï¿½ï¿½?ï¿½ï¿½ ?ï¿½ï¿½?ï¿½ï¿½?ï¿½ï¿½ ?ï¿½ï¿½?ï¿½ï¿½ ?ï¿½ï¿½?ï¿½ï¿½ì¹˜ï¿½? ?ï¿½ï¿½?ï¿½ï¿½?ï¿½ï¿½ï¿?? êµ¬ë¶„
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_3)
	{
		SW1_flag = 1;
	}

	if(GPIO_Pin == GPIO_PIN_15)
	{
		SW2_flag = 1;
	}

	if(GPIO_Pin == GPIO_PIN_4)
	{
		SW3_flag = 1;
	}

	if(GPIO_Pin == GPIO_PIN_10)
	{
		SW4_flag = 1;
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
