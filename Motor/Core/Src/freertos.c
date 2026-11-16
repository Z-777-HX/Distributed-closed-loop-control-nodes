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
#include "usart.h"
#include "drawPID.h"
#undef configMAX_PRIORITIES
#define configMAX_PRIORITIES 56

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern PID_InitTypeDef hpid;
extern osMessageQueueId_t queue;
extern struct queue_struct TxtoQueuebuf;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern char Rx_USART1_cmd[26];
extern float target_speed; 
extern int8_t NewCmd_flag;
extern float temp_kp;
extern float temp_ki;
extern float temp_kd;
extern u8g2_t u8g2;
extern int8_t actual_buf[POINT_NUM];
extern int8_t target_buf[POINT_NUM];

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
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh4,
};
/* Definitions for UartTask */
osThreadId_t UartTaskHandle;
const osThreadAttr_t UartTask_attributes = {
  .name = "UartTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh3,
};
/* Definitions for OledTask */
osThreadId_t OledTaskHandle;
const osThreadAttr_t OledTask_attributes = {
  .name = "OledTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void prvControlTask(void *argument);
void prvUartTask(void *argument);
void prvOledTask(void *argument);

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
		// #0.检查队列
		queue_struct RxfromQueuebuf = {0};
		if(osMessageQueueGet(queue, (void *)&RxfromQueuebuf, NULL, 0) == osOK)
		{
			queue_struct *temp = (queue_struct *)&RxfromQueuebuf;
			
			if(temp->cmd_flag == 1)
			{
				PID_SetTarget_Speed(&hpid, temp->queue_target_speed);
			}
			else if(temp->cmd_flag == 2)
			{
				PID_SetPID(&hpid, temp->queue_kp, temp->queue_ki, temp->queue_kd);
			}
		}
		
		// #1.获取当前速度
		float current_speed = GetCurrent_Speed(4, 11, 103);
		
		// #2.更新当前速度，将当前速度与目标速度存入绘制波形缓冲区
		PID_UpdateCurrent_Speed(&hpid, current_speed);
		
		add_to_buf(actual_buf, hpid.Current_Speed);
		add_to_buf(target_buf, hpid.Target_Speed);
		
		// #3.计算PWM，判断是否反转电机
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
		
		// #4.更新PWM占空比
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM);
		
		// #5.“睡”20ms
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
		
		// #是否有新命令需要写入队列
		if(NewCmd_flag == 1)
		{
			TxtoQueuebuf.cmd_flag = 1;
			osMessageQueuePut(queue, (void *)&TxtoQueuebuf, 0, 0);	
			
			u8g2_ClearBuffer(&u8g2);
			u8g2_DrawStr(&u8g2, 0, 12, "SET TARGET SUCCESS.");
			u8g2_SendBuffer(&u8g2);
		}
		if(NewCmd_flag == 2)
		{
			TxtoQueuebuf.cmd_flag = 2;
			osMessageQueuePut(queue, (void *)&TxtoQueuebuf, 0, 0);
			
			u8g2_ClearBuffer(&u8g2);
			u8g2_DrawStr(&u8g2, 0, 12, "SET PID SUCCESS.");
			u8g2_SendBuffer(&u8g2);
		}
		else if(NewCmd_flag == -1)
		{
			u8g2_ClearBuffer(&u8g2);
			u8g2_DrawStr(&u8g2, 0, 12, "ERROR!!");
			u8g2_SendBuffer(&u8g2);
		}
		
		NewCmd_flag = 0;
		
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
    draw_speed_curve(actual_buf, target_buf);
		osDelay(100);
  }
  /* USER CODE END prvOledTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

