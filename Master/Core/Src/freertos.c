/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "convention.h"
#include "can.h"
#include "my_u8g2.h"
#include "gpio.h"
#include "queue.h"
#include <stdio.h>
#include "usart.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern u8g2_t u8g2;
extern QueueHandle_t canRx_queue;
extern QueueHandle_t uartRx_queue;
static uint8_t motorstatus[MOTORNUM+1] = {0};
static CAN_struct canTx_struct = {0};
static uint32_t last_status_time = 0;
static uint32_t txMailbox;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UartTask */
osThreadId_t UartTaskHandle;
const osThreadAttr_t UartTask_attributes = {
  .name = "UartTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh3,
};
/* Definitions for OledTask */
osThreadId_t OledTaskHandle;
const osThreadAttr_t OledTask_attributes = {
  .name = "OledTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for HeartBestTask */
osThreadId_t HeartBestTaskHandle;
const osThreadAttr_t HeartBestTask_attributes = {
  .name = "HeartBestTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh4,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void prvUartTask(void *argument);
void prvOledTask(void *argument);
void prvCANHeartBeatTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of UartTask */
  UartTaskHandle = osThreadNew(prvUartTask, NULL, &UartTask_attributes);

  /* creation of OledTask */
  OledTaskHandle = osThreadNew(prvOledTask, NULL, &OledTask_attributes);

  /* creation of HeartBestTask */
  HeartBestTaskHandle = osThreadNew(prvCANHeartBeatTask, NULL, &HeartBestTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_prvUartTask */
/**
* @brief Function implementing the UartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_prvUartTask */
void prvUartTask(void *argument)
{
  /* USER CODE BEGIN prvUartTask */
  /* Infinite loop */
  for(;;)
  {
		// #1.检查uart队列
		static char uartTemp[50];
		while(osMessageQueueGet(uartRx_queue, (void *)uartTemp, NULL, 0) == osOK)
		{
			int temp_num = 0;
			float temp_target_speed = 0.0f;
			float temp_kp = 0.0f;
			float temp_ki = 0.0f;
			float temp_kd = 0.0f;
			
			static CAN_TxHeaderTypeDef TxHeader;
			
			if(sscanf(uartTemp, "SET%f TO MOTOR%d", &temp_target_speed, &temp_num) == 2)
			{
				if(temp_num == 1)
				{
					uint16_t u_temp_speed = (uint16_t)((int16_t)(temp_target_speed * 100.0f));
					
					canTx_struct.Can_data[0] = u_temp_speed;
					canTx_struct.Can_data[1] = (u_temp_speed) >> 8;
					
					TxHeader.StdId = CAN_ID_MOTOR1_TARGET_SPEED;
					TxHeader.IDE = CAN_ID_STD;
					TxHeader.DLC = CAN_DLC_MOTOR1_TARGET_SPEED;
					TxHeader.RTR = CAN_RTR_DATA;
					TxHeader.TransmitGlobalTime = DISABLE;
					
					HAL_CAN_AddTxMessage(&hcan, &TxHeader, canTx_struct.Can_data, &txMailbox);
				}
			}
			else if(sscanf(uartTemp, "KP%f KI%f KD%f TO MOTOR%d", &temp_kp, &temp_ki, &temp_kd, &temp_num) == 4)
			{
				if(temp_num == 1)
				{
					motorstatus[1] = 1;
					
					uint16_t kp = (uint16_t)(temp_kp * 100.0f);
					uint16_t ki = (uint16_t)(temp_ki * 100.0f);
					uint16_t kd = (uint16_t)(temp_kd * 100.0f);
					
					canTx_struct.Can_data[0] = kp;
					canTx_struct.Can_data[1] = kp >> 8;
					canTx_struct.Can_data[2] = ki;
					canTx_struct.Can_data[3] = ki >> 8;
					canTx_struct.Can_data[4] = kd;
					canTx_struct.Can_data[5] = kd >> 8;
					
					TxHeader.StdId = CAN_ID_MOTOR1_PID;
					TxHeader.IDE = CAN_ID_STD;
					TxHeader.DLC = CAN_DLC_MOTOR1_PID;
					TxHeader.RTR = CAN_RTR_DATA;
					TxHeader.TransmitGlobalTime = DISABLE;
					
					HAL_CAN_AddTxMessage(&hcan, &TxHeader, canTx_struct.Can_data, &txMailbox);
				}
			}
		}
		
		// #2.检查can队列
		static CAN_struct tempStruct = {0};
		while(osMessageQueueGet(canRx_queue, (void *)&tempStruct, NULL, 0) == osOK)
		{
			if(tempStruct.Can_Id == CAN_ID_MOTOR1_STATUS)
			{
				last_status_time = HAL_GetTick();
				motorstatus[1] = 1;
				
				static uint16_t PWM;
				static uint16_t u_current_speed;
				PWM = tempStruct.Can_data[0] + (tempStruct.Can_data[1] << 8);
				u_current_speed = tempStruct.Can_data[2] + (tempStruct.Can_data[3] << 8);
				
				float current_speed = ((float)u_current_speed) / 100.f;
				float duty_cycle = (((float)PWM) / 1799.0f) * 100.0f;
				
				printf("%.2f,%.2f\r\n", current_speed, duty_cycle);
			}
		}
		
		// #3.确认motor状态
		if(HAL_GetTick() - last_status_time >= EMERGENCY_TIME)
		{
			motorstatus[1] = 0;
		}
		
    osDelay(50);
  }
  /* USER CODE END prvUartTask */
}

/* USER CODE BEGIN Header_prvOledTask */
/**
* @brief Function implementing the OledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_prvOledTask */
void prvOledTask(void *argument)
{
  /* USER CODE BEGIN prvOledTask */
  /* Infinite loop */
  for(;;)
  {
		u8g2_ClearBuffer(&u8g2);
		
		if(motorstatus[1] == 1)
		{
			u8g2_DrawStr(&u8g2, 12, 12, "MOTOR1 RUNNING...");
			HAL_GPIO_WritePin(GPIOA, Motor1_status_flag_Pin, GPIO_PIN_RESET);
		}
		else
		{
			u8g2_DrawStr(&u8g2, 12, 12, "MOTOR1 FAULT!!");
			HAL_GPIO_WritePin(GPIOA, Motor1_status_flag_Pin, GPIO_PIN_SET);
		}
		
		u8g2_SendBuffer(&u8g2);
		
    osDelay(100);
  }
  /* USER CODE END prvOledTask */
}

/* USER CODE BEGIN Header_prvCANHeartBeatTask */
/**
* @brief Function implementing the HeartBestTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_prvCANHeartBeatTask */
void prvCANHeartBeatTask(void *argument)
{
  /* USER CODE BEGIN prvCANHeartBeatTask */
	TickType_t xlast_time = xTaskGetTickCount();
	
  /* Infinite loop */
  for(;;)
  {
		// #1.向受控节点发送心跳
		static CAN_TxHeaderTypeDef TxHeader;
		static uint8_t heart[1] = {1};
		
		TxHeader.StdId = CAN_ID_MASTER_HEARTBEAT;
		TxHeader.DLC = CAN_DLC_MASTER_HEARTBEAT;
		TxHeader.RTR = CAN_RTR_DATA;
		TxHeader.IDE = CAN_ID_STD;
		TxHeader.TransmitGlobalTime = DISABLE;
			
		HAL_CAN_AddTxMessage(&hcan, &TxHeader, heart, &txMailbox);
		
		MasterOnBoardLEDStatus();
		
		// #2.睡“50”ms
    vTaskDelayUntil(&xlast_time, pdMS_TO_TICKS(50));
  }
  /* USER CODE END prvCANHeartBeatTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

