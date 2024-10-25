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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <stdbool.h>
#include "stm32l4s5i_iot01.h"
#include "stm32l4s5i_iot01_tsensor.h"
#include "stm32l4s5i_iot01_hsensor.h"
#include "stm32l4s5i_iot01_accelero.h"
#include "stm32l4s5i_iot01_gyro.h"
#include "stm32l4s5i_iot01_psensor.h"
#include "stm32l4s5i_iot01_magneto.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#ifdef __GNUC__
#define PUTCHAR_PROTO int __io_putchar(int ch)
#else
#define PUTCHAR_PROTO int fputc(int ch, FILE *f)
#endif
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static uint8_t msg1[] = "______ Multiple Sensor value measurement Tian/Kai ______\n\n\r";
static uint8_t msg2[] = "======> Initializing Sensors\r\n\n";
static uint8_t msg3[] = "======> Sensors initialized!\r\n\n";
static bool not_Pressed = false;
static bool Sensor_Change_Cooldown = false; //recent sensor change
//const static uint8_t App_Modes[6] = {"T","H","A","G","P","M"};//sensor list with capital initials
static uint8_t current_mode;
static uint8_t running_mode;

float temp_buff = 0;
float hum_buff = 0;
float pressure_buff = 0;

static int16_t pAcceloData[3] ;
static int16_t pMagData[3] ;
float pGyroData[3] ;
static uint8_t str_buff[50] = "";
//uint8_t str_test[100] = "";


PUTCHAR_PROTO
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

void Button_Click_Poll(){
	for(;;){
		osDelay(5);
		//snprintf(str_test,100,"ID1 %d\n\r",(uint8_t)Sensor_Change_Cooldown);
		//HAL_UART_Transmit(&huart1, str_test,100,HAL_MAX_DELAY);
		//str_test[0] = (char)HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13);//button
		//HAL_UART_Transmit(&huart1, str_test,1,HAL_MAX_DELAY);
		not_Pressed = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13);
		if(!not_Pressed&&(!Sensor_Change_Cooldown)){
			HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);//toggle led
			Sensor_Change_Cooldown = true;
			switch(current_mode){
				case 'T': //T
					current_mode = (uint8_t)'H';
					break;
				case 'H': //H
					current_mode = (uint8_t)'A';
					break;
				case 'A': //A
					current_mode = (uint8_t)'G';
					break;
				case 'G': //G
					current_mode = (uint8_t)'P';
					break;
				case 'P':
					current_mode = (uint8_t)'M';
					break;
				case 'M':
					current_mode = (uint8_t)'T';
					break;
			}
		}
	}
}

void Flag_Reset(){
	for(;;){
		osDelay(10);
		//snprintf(str_test,100,"ID2\n\r");
		//HAL_UART_Transmit(&huart1, str_test,1,HAL_MAX_DELAY);
		if(Sensor_Change_Cooldown){
			osDelay(500);
			Sensor_Change_Cooldown = false;
		}
	}
}

void Sensor_SW_Poll(){
	for(;;){
		osDelay(100);
		//snprintf(str_test,100,"ID3\n\r%d\n\r",(int)Sensor_Change_Cooldown);
		//HAL_UART_Transmit(&huart1, str_test,100,HAL_MAX_DELAY);
		//HAL_UART_Transmit(&huart1, &current_mode,1,HAL_MAX_DELAY);
		if((!Sensor_Change_Cooldown)&&(current_mode != running_mode)){
			running_mode = current_mode;
		}
		switch(running_mode){
			case 'T':
				temp_buff = BSP_TSENSOR_ReadTemp();
				break;
			case 'H':
				hum_buff = BSP_HSENSOR_ReadHumidity();
				break;
			case 'A':
				BSP_ACCELERO_AccGetXYZ(pAcceloData);
				break;
			case 'G':
				BSP_GYRO_GetXYZ(pGyroData);
				break;
			case 'P':
				pressure_buff =  BSP_PSENSOR_ReadPressure();
				break;
			case 'M':
				BSP_MAGNETO_GetXYZ(pMagData);
				break;
		}
	}
}

void UART_Print_Data(){
	for(;;){
		osDelay(1000);
		//snprintf(str_test,100,"ID4\n\r");
		//HAL_UART_Transmit(&huart1, str_test,100,HAL_MAX_DELAY);
			switch(running_mode){
				case 'T':
					snprintf((char*)str_buff,50,"Temperature = %d.%02d       \n\r",(int)temp_buff, (int)((temp_buff-(int)temp_buff)*100));
					HAL_UART_Transmit(&huart1, str_buff,50,HAL_MAX_DELAY);
					break;
				case 'H':
					snprintf((char*)str_buff,50,"Humidity = %d.%02d          \n\r",(int)hum_buff, (int)((hum_buff-(int)hum_buff)*100));
					HAL_UART_Transmit(&huart1, str_buff,50,HAL_MAX_DELAY);
					break;
				case 'A':
					snprintf((char*)str_buff,50,"Acceleration = (%d, %d, %d) \n\r",pAcceloData[0], pAcceloData[1],pAcceloData[2]);
					HAL_UART_Transmit(&huart1, str_buff,50,HAL_MAX_DELAY);
					break;
				case 'G':
					snprintf((char*)str_buff,50,"Gyro = (%d, %d, %d)         \n\r",(int)pGyroData[0], (int)pGyroData[1],(int)pGyroData[2]);
					HAL_UART_Transmit(&huart1, str_buff,50,HAL_MAX_DELAY);
					break;
				case 'P':
					snprintf((char*)str_buff,50,"Pressure = %d.%02d          \n\r",(int)pressure_buff, (int)((pressure_buff-(int)pressure_buff)*100));
					HAL_UART_Transmit(&huart1, str_buff,50,HAL_MAX_DELAY);
					break;
				case 'M':
					snprintf((char*)str_buff,50,"Mageneto = (%d, %d, %d)     \n\r",pMagData[0], pMagData[1],pMagData[2]);
					HAL_UART_Transmit(&huart1, str_buff,50,HAL_MAX_DELAY);
					break;

		}
	}
}

osThreadId id1_handle,id2_handle,id3_handle,id4_handle;

osThreadDef(id1, Button_Click_Poll, osPriorityAboveNormal,1,128);
osThreadDef(id2, Flag_Reset, osPriorityNormal,1,128);
osThreadDef(id3, Sensor_SW_Poll, osPriorityAboveNormal,1,128);
osThreadDef(id4, UART_Print_Data, osPriorityAboveNormal,1,256);

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  printf((char*)msg1);
  printf((char*)msg2);
  BSP_HSENSOR_Init();
  BSP_TSENSOR_Init();
  BSP_ACCELERO_Init();
  BSP_GYRO_Init();
  BSP_PSENSOR_Init();
  BSP_MAGNETO_Init();
  current_mode = 'T';
  running_mode = ' ';
  printf((char*)msg3);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  //osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 129);
  //defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  id1_handle = osThreadCreate(osThread(id1),NULL );
  id2_handle = osThreadCreate(osThread(id2),NULL );
  id3_handle = osThreadCreate(osThread(id3),NULL );
  id4_handle = osThreadCreate(osThread(id4),NULL );
  if(id1_handle==NULL)printf("ID1 creation failed/n/r");
  if(id2_handle==NULL)printf("ID2 creation failed/n/r");
  if(id3_handle==NULL)printf("ID3 creation failed/n/r");
  if(id4_handle==NULL)printf("ID4 creation failed/n/r");
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x307075B1;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
