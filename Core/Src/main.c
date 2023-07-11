/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

pid_type_def motor_pid_1;            // 澹版槑PID鏁版嵁缁撴�?�锟�??????
pid_type_def motor_pid_2;
const motor_measure_t* motor_data_1; // 澹版槑鐢垫満缁撴瀯浣撴寚锟�???
const motor_measure_t* motor_data_2;
const fp32 PID[3] = {6.0, 0.3, 0.1 }; // P,I,D鍙傛�???

int16_t motor_target_f_left;
int16_t motor_target_f_right;
int16_t motor_wheel_left;
int16_t motor_wheel_right;
float motor_low_pass_alpha = 0.84;

int last_tick, current_tick;

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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  can_filter_init();
  PID_init(&motor_pid_1, PID_POSITION, PID, 16000, 2000); // PID缁撴瀯浣擄紝PID璁＄畻妯″紡锛孭ID鍙傛暟锛屾渶澶э拷?锟斤紝锟�??????澶锟�???
  PID_init(&motor_pid_2, PID_POSITION, PID, 16000, 2000); // PID缁撴瀯浣擄紝PID璁＄畻妯″紡锛孭ID鍙傛暟锛屾渶澶э拷?锟斤紝锟�??????澶锟�???
  motor_data_1 = get_chassis_motor_measure_point(0);      // 鑾峰彇ID锟�???1鍙风殑鐢垫満鏁版嵁鎸囬拡
  motor_data_2 = get_chassis_motor_measure_point(1);      // 鑾峰彇ID锟�???2鍙风殑鐢垫満鏁版嵁鎸囬拡

   bsp_uart1_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	bsp_uart1_rx();
	  current_tick = HAL_GetTick();

	  if((current_tick-last_tick)>10)
	  {
		  last_tick = current_tick;

		  bsp_uart1_set_wheel_speed(motor_wheel_left, motor_wheel_right);
		  bsp_uart1_tx();
	  }

	/////////////////////////////////
//	CAN_get_Motor_Data(&hcan1);
	/////////////////////////////////
//	motor_target_f_left = 1000;
//	motor_target_f_right = 1000;

//	motor_wheel_left =  (float)motor_data_1->speed_rpm *motor_low_pass_alpha + (1-motor_low_pass_alpha) * motor_wheel_left; //一阶低通滤波
//	motor_wheel_right =  (float)motor_data_2->speed_rpm *motor_low_pass_alpha + (1-motor_low_pass_alpha) * motor_wheel_right; //一阶低通滤波
	  motor_wheel_left =  motor_data_1->speed_rpm;
	  motor_wheel_right =  motor_data_2->speed_rpm;

	bsp_uart_get_wheel_speed(&motor_target_f_left, &motor_target_f_right);

	PID_calc(&motor_pid_1,(float)motor_wheel_left, (float)motor_target_f_left); // 璁＄畻鐢垫満pid杈撳嚭锛孭ID缁撴瀯浣擄紝瀹為檯閫熷害锛岃�?�氾�????锟藉�???
	PID_calc(&motor_pid_2, (float)motor_wheel_right, -(float)motor_target_f_right); // 璁＄畻鐢垫満pid杈撳嚭锛孭ID缁撴瀯浣擄紝瀹為檯閫熷害锛岃�?�氾�????锟藉�???

	CAN_cmd_chassis(motor_pid_1.out, motor_pid_2.out, 0, 0);            // 鍙戯�????锟�?�绠楀悗鐨勬帶鍒剁數娴佺粰鐢垫�???1鍜岀數锟�??????2锛岀數锟�??????3锟�???4鍦ㄨ繖閲屼负0

	HAL_Delay(2);
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
  RCC_OscInitStruct.PLL.PLLM = 6;
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

/* USER CODE BEGIN 4 */

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
