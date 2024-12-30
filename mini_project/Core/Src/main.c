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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_gyroscope.h"
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
/* Definitions for ReadAndProcess */
osThreadId_t ReadAndProcessHandle;
const osThreadAttr_t ReadAndProcess_attributes = {
  .name = "ReadAndProcess",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Output */
osThreadId_t OutputHandle;
const osThreadAttr_t Output_attributes = {
  .name = "Output",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for ZPID */
osThreadId_t ZPIDHandle;
const osThreadAttr_t ZPID_attributes = {
  .name = "ZPID",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for XPID */
osThreadId_t XPIDHandle;
const osThreadAttr_t XPID_attributes = {
  .name = "XPID",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* USER CODE BEGIN PV */
static int target_height = 30;
static int current_height = 30;
static int target_width = 125;
static int current_width = 125;
static int score = 0;
const float Kp = 0.1;
const float Ki = 0.001;
const float Kd = 0.05;
int end = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void ReadAndProcess_Handle(void *argument);
void Output_handle(void *argument);
void ZPID_handle(void *argument);
void XPID_handle(void *argument);

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
  /* USER CODE BEGIN 2 */
  BSP_GYRO_Init();
  BSP_LCD_Init();
  BSP_LCD_LayerDefaultInit(1, SDRAM_DEVICE_ADDR);
  BSP_LCD_SelectLayer(1);
  BSP_LCD_DisplayOn();
  BSP_LCD_Clear(LCD_COLOR_BLUE);
  BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_DisplayStringAtLine(1, (uint8_t*)"Score: 0");
  BSP_LCD_FillCircle(125, 160, 30);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* creation of ReadAndProcess */
  ReadAndProcessHandle = osThreadNew(ReadAndProcess_Handle, NULL, &ReadAndProcess_attributes);

  /* creation of Output */
  OutputHandle = osThreadNew(Output_handle, NULL, &Output_attributes);

  /* creation of ZPID */
  ZPIDHandle = osThreadNew(ZPID_handle, NULL, &ZPID_attributes);

  /* creation of XPID */
  XPIDHandle = osThreadNew(XPID_handle, NULL, &XPID_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LED_3_Pin|LED_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_3_Pin LED_4_Pin */
  GPIO_InitStruct.Pin = LED_3_Pin|LED_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_ReadAndProcess_Handle */
/**
  * @brief  Function implementing the ReadAndProcess thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ReadAndProcess_Handle */
void ReadAndProcess_Handle(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  float pfData[3];
  /* Infinite loop */
  for(;;)
  {
	if(end)
	{
		osThreadTerminate(ReadAndProcessHandle);
	}
	BSP_GYRO_GetXYZ(pfData);
	pfData[1] = pfData[1] * 17.5 * 0.001;
	pfData[2] = pfData[2] * 17.5 * 0.001;
	if(target_height == 30 && current_height <= -(int)pfData[2])
	{
		target_height*=1.5;
		target_width+=pfData[1];
		if(target_height > 100)
		{
			target_height = 100;
		}
		score+=1;
	}
	if(current_height >= target_height)
	{
		target_height = 30;
	}
    osDelay(100);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Output_handle */
/**
* @brief Function implementing the Output thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Output_handle */
void Output_handle(void *argument)
{
  /* USER CODE BEGIN Output_handle */
  char data[20];
  char usb[20];
  int old_score = 0;
  /* Infinite loop */
  for(;;)
  {
	if(end)
	{
		BSP_LCD_Clear(LCD_COLOR_BLUE);
		BSP_LCD_DisplayStringAtLine(1, (uint8_t*)"GAMEOVER");
		HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, 0);
		HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, 1);
		osThreadTerminate(OutputHandle);
	}
	if(score != old_score)
	{
		old_score = score;
		HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, 1);
	}
	else
	{
		HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, 0);
	}
	sprintf(data, "Score: %d", score);
	BSP_LCD_DisplayStringAtLine(1, (uint8_t*)data);
	sprintf(usb, "%d\n", current_height);
	CDC_Transmit_HS((uint8_t*)usb, 20);
	for(int i = 2; i <= 11; i++)
	{
		BSP_LCD_ClearStringLine(i);
	}
	BSP_LCD_FillCircle(current_width, 160, current_height);
    osDelay(100);
  }
  /* USER CODE END Output_handle */
}

/* USER CODE BEGIN Header_ZPID_handle */
/**
* @brief Function implementing the ZPID thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ZPID_handle */
void ZPID_handle(void *argument)
{
  /* USER CODE BEGIN ZPID_handle */
  int old_error = 0;
  int sum_error = 0;
  /* Infinite loop */
  for(;;)
  {
	int error = target_height - current_height;
	float P = Kp * error;
	float D = Kd * (error - old_error) / 100;
	sum_error+=error;
	float I = Ki * sum_error * 100;
	old_error = error;
	current_height += (int)(P + D + I);
	if(current_height < 30)
	{
		end = 1;
		osThreadTerminate(ZPIDHandle);
	}
    osDelay(100);
  }
  /* USER CODE END ZPID_handle */
}

/* USER CODE BEGIN Header_XPID_handle */
/**
* @brief Function implementing the XPID thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_XPID_handle */
void XPID_handle(void *argument)
{
  /* USER CODE BEGIN XPID_handle */
  int old_error = 0;
  int sum_error = 0;
  /* Infinite loop */
  for(;;)
  {
	if(end)
	{
		osThreadTerminate(XPIDHandle);
	}
	int error = target_width - current_width;
	float P = Kp * error;
	float D = Kd * (error - old_error) / 100;
	sum_error+=error;
	float I = Ki * sum_error * 100;
	old_error = error;
	int temp = current_width + (int)(P + D + I);
	if(temp + current_height > 210 || temp - current_height < 0)
	{
	  target_width = 125;
	}
	else
	{
	  current_width = temp;
	}
    osDelay(100);
  }
  /* USER CODE END XPID_handle */
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
