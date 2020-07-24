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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
uint8_t usbRxData[APP_RX_DATA_SIZE] ;

//The number should be the same as APP_RX_DATA_SIZE
#include "nbt.h"
#include <math.h>
#include "cpp_main.h"
#include "ringbuffer.h"
#include "mpu6050_usr.h"
#include <stdbool.h>

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
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

osThreadId defaultTaskHandle;
osThreadId IMUtaskHandle;
osThreadId wheelControltasHandle;
osThreadId sensorsTaskHandle;
/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef r_wheelHeader;
CAN_TxHeaderTypeDef l_wheelHeader;
CAN_TxHeaderTypeDef r_wheelHeader1;
CAN_TxHeaderTypeDef l_wheelHeader1;
CAN_FilterTypeDef sFilterConfig;
CAN_RxHeaderTypeDef wheel_RxHeader;

uint32_t can;

uint32_t TxMailbox;
uint32_t leftCount, rightCount;
uint8_t ctrl = 0x00;
int16_t allData[6];
uint8_t control_data_left[2];
uint8_t control_data_right[2];
int8_t sideDataRight;
int8_t speedDataRight;
int8_t sideDataLeft;
int8_t speedDataLeft;
uint8_t sensorData1 = 0;
uint8_t sensorData2 = 0;
uint8_t sensorData3 = 0;
uint8_t sensorData4 = 0;
uint8_t sensorData5 = 0;
uint8_t sensorData6 = 0;
uint8_t speedRXDataRight;
uint8_t speedRXDataLeft;
uint8_t sideRXDataRight;
uint8_t sideRXDataLeft;
uint8_t canRXData[8];
float gyroX;
float gyroY;
float gyroZ;
float accelX;
float accelY;
float accelZ;
uint8_t r_wheel_data[2];
uint8_t l_wheel_data[2];
uint8_t r[2];
uint8_t side = 0;
uint8_t pwm = 0;
uint32_t f;
uint8_t nh_connected = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);
void StartTask04(void const * argument);

/* USER CODE BEGIN PFP */
void rpm_right_handler(void);
void rpm_left_handler(void);
void laser_sensor_handler_1(void);
void laser_sensor_handler_2(void);
void laser_sensor_handler_3(void);
void laser_sensor_handler_4(void);
void laser_sensor_handler_5(void);
void laser_sensor_handler_6(void);
void accel_handler(void);
void gyro_handler(void);
void spinOnce(void);
void init_ROS(void);
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
  MX_CAN1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  //sensor_ini();
  MPU6050_init();
  HAL_Delay(1000);
  init_ROS();
  HAL_Delay(1000);

  r_wheelHeader.DLC = 2;
  r_wheelHeader.IDE = CAN_ID_STD;
  r_wheelHeader.RTR = CAN_RTR_DATA;
  r_wheelHeader.StdId = 0xF;

  l_wheelHeader.DLC = 2;
  l_wheelHeader.IDE = CAN_ID_STD;
  l_wheelHeader.RTR = CAN_RTR_DATA;
  l_wheelHeader.StdId = 0x1F;


  r_wheelHeader1.DLC = 2;
  r_wheelHeader1.IDE = CAN_ID_STD;
  r_wheelHeader1.RTR = CAN_RTR_DATA;
  r_wheelHeader1.StdId = 0x2F;

  l_wheelHeader1.DLC = 2;
  l_wheelHeader1.IDE = CAN_ID_STD;
  l_wheelHeader1.RTR = CAN_RTR_DATA;
  l_wheelHeader1.StdId = 0x3F;

//  right_wheel_RxHeader.DLC = 1;
//  right_wheel_RxHeader.IDE = CAN_ID_STD;
//  right_wheel_RxHeader.RTR = CAN_RTR_DATA;
//  right_wheel_RxHeader.StdId = 0xFF;
//
//  left_wheel_RxHeader.DLC = 1;
//  left_wheel_RxHeader.IDE = CAN_ID_STD;
//  left_wheel_RxHeader.RTR = CAN_RTR_DATA;
//  left_wheel_RxHeader.StdId = 0x3F;

  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  sFilterConfig.FilterIdHigh = 0;
  sFilterConfig.FilterIdLow = 0;
  sFilterConfig.FilterMaskIdHigh = 0;
  sFilterConfig.FilterMaskIdLow = 0;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  //sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
  sFilterConfig.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);

  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of IMUtask */
  osThreadDef(IMUtask, StartTask02, osPriorityNormal, 0, 128);
  IMUtaskHandle = osThreadCreate(osThread(IMUtask), NULL);

  /* definition and creation of wheelControltas */
  osThreadDef(wheelControltas, StartTask03, osPriorityHigh, 0, 128);
  wheelControltasHandle = osThreadCreate(osThread(wheelControltas), NULL);

  /* definition and creation of sensorsTask */
  osThreadDef(sensorsTask, StartTask04, osPriorityNormal, 0, 128);
  sensorsTaskHandle = osThreadCreate(osThread(sensorsTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {

	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &wheel_RxHeader, canRXData);
	if (wheel_RxHeader.StdId == 0xFFF) {

	}
	else if (wheel_RxHeader.StdId == 0x7F) {
		speedRXDataLeft = canRXData[0];
		sideRXDataLeft = canRXData[1];
	}
	else if (wheel_RxHeader.StdId == 0x3F) {
		speedRXDataRight = canRXData[0];
		sideRXDataRight = canRXData[1];
	}
	else if (wheel_RxHeader.StdId == 0x3D) {
		sensorData1 = canRXData[0];
		sensorData2 = canRXData[1];
		sensorData3 = canRXData[2];
		sensorData4 = canRXData[3];
		sensorData5 = canRXData[4];
		sensorData6 = canRXData[5];
	}
	wheel_RxHeader.StdId = 0x000;
}
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
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  gyro_handler();
	  osDelay(5);
	  accel_handler();
	  osDelay(5);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
	  MPU6050_getAllData(allData);
	  osDelay(20);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the wheelControltas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  for(;;)
  {
//		r_wheel_data[0] = sideDataRight;
//		r_wheel_data[1] = speedDataRight;
//		l_wheel_data[0] = sideDataLeft;
//		l_wheel_data[1] = speedDataLeft;
	  	r_wheel_data[0] = 0;
	  	r_wheel_data[1] = 30;
	  	l_wheel_data[0] = 1;
	  	l_wheel_data[1] = 30;
		HAL_CAN_AddTxMessage(&hcan1, &l_wheelHeader, r_wheel_data, &TxMailbox);
		osDelay(2);
		HAL_CAN_AddTxMessage(&hcan1, &r_wheelHeader, r_wheel_data, &TxMailbox);
		osDelay(2);
		HAL_CAN_AddTxMessage(&hcan1, &l_wheelHeader1, l_wheel_data, &TxMailbox);
		osDelay(2);
		HAL_CAN_AddTxMessage(&hcan1, &r_wheelHeader1, l_wheel_data, &TxMailbox);
		osDelay(2);
		//if (HAL_CAN_StateTypeDef == HAL_OK) {
			//can++;
		//}
//		osDelay(2);
//		rpm_right_handler();
//		osDelay(5);
//		rpm_left_handler();
//		osDelay(5);
		spinOnce();
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the sensorsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void const * argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Infinite loop */
  for(;;)
  {

	laser_sensor_handler_1();
	osDelay(50);
//	laser_sensor_handler_2();
//	osDelay(5);
//	laser_sensor_handler_3();
//	osDelay(5);
//	laser_sensor_handler_4();
//	osDelay(5);
//	laser_sensor_handler_5();
//	osDelay(5);
//	laser_sensor_handler_6();
//	osDelay(5);

  }
  /* USER CODE END StartTask04 */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
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
