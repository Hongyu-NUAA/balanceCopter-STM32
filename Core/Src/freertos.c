/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "CAN_receive.h"
#include "bsp_can.h"
#include "pid.h"

#include "bsp_uart.h"
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
/* USER CODE BEGIN Variables */

int16_t motor_target_f_left_int;
int16_t motor_target_f_right_int;

int16_t motor_wheel_left_int;
int16_t motor_wheel_right_int;

const motor_measure_t* motor_data_left; // 澹版槑鐢垫満缁撴瀯浣撴寚锟�???
const motor_measure_t* motor_data_right;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
osThreadId_t canTaskHandle;
const osThreadAttr_t canTask_attributes = {
  .name = "canTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
void canTask(void *argument);

osThreadId_t uartTaskHandle;
const osThreadAttr_t uartTask_attributes = {
  .name = "uartTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
void uartTask(void *argument);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  canTaskHandle = osThreadNew(canTask, NULL, &canTask_attributes);
  uartTaskHandle = osThreadNew(uartTask, NULL, &uartTask_attributes);

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
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void canTask(void *argument)
{
	can_filter_init();

	motor_data_left = get_chassis_motor_measure_point(0);      // 鑾峰彇ID锟�???1鍙风殑鐢垫満鏁版嵁鎸囬拡
	motor_data_right = get_chassis_motor_measure_point(1);      // 鑾峰彇ID锟�???2鍙风殑鐢垫満鏁版嵁鎸囬拡

	motor_target_f_left_int = 0;
	motor_target_f_right_int = 0;

  for(;;)
  {
	  motor_wheel_left_int =  motor_data_left->speed_rpm;
	  motor_wheel_right_int =  -motor_data_right->speed_rpm;

	  bsp_uart_get_target_wheel_speed(&motor_target_f_left_int, &motor_target_f_right_int);

	  CAN_cmd_chassis(motor_target_f_left_int, -motor_target_f_right_int, 0, 0);

	  osDelay(2);
  }
  /* USER CODE END StartDefaultTask */
}

void uartTask(void *argument)
{
   bsp_uart1_init();


  for(;;)
  {
	  bsp_uart1_get_feedback_wheel_speed(motor_data_left->speed_rpm, -motor_data_right->speed_rpm);
	  bsp_uart1_tx();

	  osDelay(3);
  }
  /* USER CODE END StartDefaultTask */
}
/* USER CODE END Application */

