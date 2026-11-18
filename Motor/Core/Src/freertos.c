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
#include <stdio.h>
#include "pid.h"
#include "encoder.h"
#include "u8g2.h"
#include "tim.h"
//#include "drawPID.h"
#include "convention.h"
#include "can.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern PID_InitTypeDef hpid;
extern osMessageQueueId_t can_queue;
extern osMessageQueueId_t uart_queue;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
static uint8_t UartNewCmd_flag = 0;
static uint8_t CanNewCmd_flag = 0;
static uint8_t Heart_flag = 0;
extern u8g2_t u8g2;
//static int8_t actual_buf[POINT_NUM];
//static int8_t target_buf[POINT_NUM];
static uint32_t last_HeartBeat = 0;
static CAN_struct CAN_Tx = {0};

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
/* Definitions for ControlTask */
osThreadId_t ControlTaskHandle;
const osThreadAttr_t ControlTask_attributes = {
  .name = "ControlTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh4,
};
/* Definitions for UartTask */
osThreadId_t UartTaskHandle;
const osThreadAttr_t UartTask_attributes = {
  .name = "UartTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh3,
};
/* Definitions for OledTask */
osThreadId_t OledTaskHandle;
const osThreadAttr_t OledTask_attributes = {
  .name = "OledTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for StatusReport */
osThreadId_t StatusReportHandle;
const osThreadAttr_t StatusReport_attributes = {
  .name = "StatusReport",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh2,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void prvControlTask(void *argument);
void prvUartTask(void *argument);
void prvOledTask(void *argument);
void prvStatusReport(void *argument);

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

  /* creation of ControlTask */
  ControlTaskHandle = osThreadNew(prvControlTask, NULL, &ControlTask_attributes);

  /* creation of UartTask */
  UartTaskHandle = osThreadNew(prvUartTask, NULL, &UartTask_attributes);

  /* creation of OledTask */
  OledTaskHandle = osThreadNew(prvOledTask, NULL, &OledTask_attributes);

  /* creation of StatusReport */
  StatusReportHandle = osThreadNew(prvStatusReport, NULL, &StatusReport_attributes);

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
		
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_prvControlTask */
/**
* @brief Function implementing the ControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_prvControlTask */
void prvControlTask(void *argument)
{
  /* USER CODE BEGIN prvControlTask */
	TickType_t xlast_time = xTaskGetTickCount();
	
  /* Infinite loop */
  for(;;)
  {
		// #1.检查can队列
		CAN_struct RxfromQueuebufStruct = {0};
		while(osMessageQueueGet(can_queue, (void *)&RxfromQueuebufStruct, NULL, 0) == osOK)
		{
			if(RxfromQueuebufStruct.Can_Id == CAN_ID_HEARTBEAT)
			{
				last_HeartBeat = HAL_GetTick();
				Heart_flag = 1;
			}
			
			if(Heart_flag) // 心跳是否正常
			{
				if(RxfromQueuebufStruct.Can_Id == CAN_ID_TARGET_SPEED)
				{
					uint16_t u_target_speed = 0;
					
					u_target_speed = RxfromQueuebufStruct.Can_data[0] + (RxfromQueuebufStruct.Can_data[1] << 8);
					
					float target_speed = ((float)((int16_t)u_target_speed)) / 100.0f;
					
					PID_SetTarget_Speed(&hpid, target_speed);
					
					CanNewCmd_flag++;
				}
				else if(RxfromQueuebufStruct.Can_Id == CAN_ID_PID)
				{
					uint16_t temp_kp = 0, temp_ki = 0, temp_kd = 0;
					
					temp_kp = RxfromQueuebufStruct.Can_data[0] + (RxfromQueuebufStruct.Can_data[1] << 8);
					temp_ki = RxfromQueuebufStruct.Can_data[2] + (RxfromQueuebufStruct.Can_data[3] << 8);
					temp_kd = RxfromQueuebufStruct.Can_data[4] + (RxfromQueuebufStruct.Can_data[5] << 8);
					
					float can_kp = ((float)temp_kp) / 100.0f;
					float can_ki = ((float)temp_ki) / 100.0f;
					float can_kd = ((float)temp_kd) / 100.0f;
					
					PID_SetPID(&hpid, can_kp, can_ki ,can_kd);
					
					CanNewCmd_flag++;
				}
			}
		}
		
		if(HAL_GetTick() - last_HeartBeat >= EMERGENCY_TIME) // 长时间心跳丢失
		{
			Heart_flag = 0;
		}
		
		// #2.检查uart队列
		char RxfromQueuebufArr[30];
		while(osMessageQueueGet(uart_queue, (void *)RxfromQueuebufArr, NULL, 0) == osOK)
		{
			float temp_kp = 0.0f;
			float temp_ki = 0.0f;
			float temp_kd = 0.0f;
			float target_speed = 0.0f;
			
			if(sscanf(RxfromQueuebufArr, "SET %f", &target_speed) == 1)
			{
				PID_SetTarget_Speed(&hpid, target_speed);
				UartNewCmd_flag++;
			}
			else if(sscanf(RxfromQueuebufArr, "KP%f KI%f KD%f", &temp_kp, &temp_ki, &temp_kd) == 3)
			{
				PID_SetPID(&hpid, temp_kp, temp_ki, temp_kd);
				UartNewCmd_flag++;
			}
		}
		
		// #3.获取当前速度
		float current_speed = GetCurrent_Speed(4, 11, 103);
		
		// #4.更新当前速度
		PID_UpdateCurrent_Speed(&hpid, current_speed);
		
//		add_to_buf(actual_buf, hpid.Current_Speed);
//		add_to_buf(target_buf, hpid.Target_Speed);
		
		
		// #5.计算PWM，判断是否反转电机
		int16_t PWM = PID_Calc(&hpid);
		
		if(PWM >= 0.0f)
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		}
		else
		{
			PWM = -PWM;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
		}
		
		// #6.更新PWM占空比
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM);
		
		// #7.“睡”20ms
		vTaskDelayUntil(&xlast_time, pdMS_TO_TICKS(20));
	}
  
  /* USER CODE END prvControlTask */
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
    printf("%.2f,%.2f\r\n", hpid.Current_Speed, hpid.Target_Speed);		
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
		// #1.是否有来自Master命令
		if(CanNewCmd_flag > 0)
		{
			u8g2_ClearBuffer(&u8g2);
			u8g2_DrawStr(&u8g2, 0, 12, "CAN CMD SUCCESS.");
			u8g2_SendBuffer(&u8g2);
			
			CanNewCmd_flag = 0;
		}
		
		if(!Heart_flag) // 心跳丢失
		{
			u8g2_ClearBuffer(&u8g2);
			u8g2_DrawStr(&u8g2, 0, 12, "MASTER ERROR!!");
			u8g2_DrawStr(&u8g2, 0, 24, "MASTER ERROR!!");
			u8g2_DrawStr(&u8g2, 0, 36, "MASTER ERROR!!");
			u8g2_DrawStr(&u8g2, 0, 60, "--ONLY UART NOW.");
			u8g2_SendBuffer(&u8g2);
		}
		else
		{
			u8g2_ClearBuffer(&u8g2);
			u8g2_DrawStr(&u8g2, 0, 12, "MASTER RUNNING...");
			u8g2_DrawStr(&u8g2, 0, 24, "MASTER RUNNING...");
			u8g2_DrawStr(&u8g2, 0, 36, "MASTER RUNNING...");
			u8g2_DrawStr(&u8g2, 0, 60, "--MASTER OR UART.");
			u8g2_SendBuffer(&u8g2);
		}
		
		// #2.是否有来自串口新命令
		if(UartNewCmd_flag)
		{
			u8g2_ClearBuffer(&u8g2);
			u8g2_DrawStr(&u8g2, 0, 12, "UART CMD SUCCESS.");
			u8g2_SendBuffer(&u8g2);
			
			UartNewCmd_flag = 0;
		}
//    draw_speed_curve(actual_buf, target_buf);
		osDelay(100);
  }
  /* USER CODE END prvOledTask */
}

/* USER CODE BEGIN Header_prvStatusReport */
/**
* @brief Function implementing the StatusReport thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_prvStatusReport */
void prvStatusReport(void *argument)
{
  /* USER CODE BEGIN prvStatusReport */
  /* Infinite loop */
  for(;;)
  {
		static CAN_TxHeaderTypeDef TxHeader;
		
		TxHeader.StdId = CAN_ID_STATUS;
		TxHeader.IDE = CAN_ID_STD;
		TxHeader.DLC = CAN_DLC_STATUS;
		TxHeader.RTR = CAN_RTR_DATA;
		TxHeader.TransmitGlobalTime = DISABLE;
		
		uint16_t PWM = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1);
		uint16_t u_current_speed = (uint16_t)(hpid.Current_Speed * 100);
		
		CAN_Tx.Can_data[0] = PWM;
		CAN_Tx.Can_data[1] = PWM >> 8;
		CAN_Tx.Can_data[2] = u_current_speed;
		CAN_Tx.Can_data[3] = u_current_speed >> 8;
		
		static uint32_t txMailbox;
		
		HAL_CAN_AddTxMessage(&hcan, &TxHeader, CAN_Tx.Can_data, &txMailbox);
    osDelay(50);
  }
  /* USER CODE END prvStatusReport */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

