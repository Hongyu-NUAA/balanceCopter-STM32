#include "bsp_uart.h"

#include "main.h"
#include "stdio.h"
#include "string.h"

float wheel_left_target;
float wheel_right_target;

//uint8_t UART1_RX_BUF[RM_UART_MAX_LEN]; // 缓存数组
//uint8_t UART1_RX_DATA;

uint8_t UART1_RX_BUF[RM_UART_MAX_LEN]; // 缓存数组
uint8_t UART1_RX_LEN;                  // 缓存数组长度

union apm_2_stm32_union apm_2_stm32;
union stm32_2_apm_union stm32_2_apm;

///////////////////////////////////////////////////////////////////////////
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    if(huart ->Instance == USART1)
//    {
//    	bsp_uart_parse(UART1_RX_DATA);
//		//等待下一次接收中断
//		HAL_UART_Receive_IT(huart,&UART1_RX_DATA,1);
//    }
//}
extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart1;

void bsp_uart1_handler(void)
{
    if ((__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET)) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);                           // 清除标志位
        HAL_UART_DMAStop(&huart1);                                    // 停止DMA接收，防止数据出错

        UART1_RX_LEN = 200 - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);  // 获取DMA中传输的数据个数

        memcpy(apm_2_stm32.bits, UART1_RX_BUF, UART1_RX_LEN);           // 更新数据

        HAL_UART_Receive_DMA(&huart1, UART1_RX_BUF, RM_UART_MAX_LEN); // 打开DMA接收，数据存入 UART1_RX_BUF 数组中
    }
}

///////////////////////////////////////////////////////////////////////////
void bsp_uart1_init(void)
{
//	//开启接收中断
//	HAL_UART_Receive_IT(&huart1, &UART1_RX_DATA, 1);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);                  // 开启串口空闲中断，必须调用
    HAL_UART_Receive_DMA(&huart1, UART1_RX_BUF, RM_UART_MAX_LEN); // 启动DMA接收
}

///////////////////////////////////////////////////////////////////////////
void bsp_uart_parse(uint8_t data)
{
	static uint8_t rx_step=0;
	static uint8_t rx_count;
	switch(rx_step)
	{
	case 0:
		if(data == 0xAA)
		{
			rx_step = 1;
			UART1_RX_BUF[rx_count++] = data;
		} else {
			rx_step=0;
			rx_count=0;
		}
		break;

	case 1:
		if(data == 0xAF)
		{
			rx_step = 2;
			UART1_RX_BUF[rx_count++] = data;
		} else {
			rx_step=0;
			rx_count=0;
		}
		break;


	case 2:
		if(data == sizeof(struct apm_2_stm32_struct))
		{
			rx_step = 3;
			UART1_RX_BUF[rx_count++] = data;
		} else {
			rx_step=0;
			rx_count=0;
		}
		break;

	case 3:
		UART1_RX_BUF[rx_count++] = data;

		if(rx_count >= sizeof(struct apm_2_stm32_struct))
		{
			memcpy(apm_2_stm32.bits, UART1_RX_BUF, sizeof(struct apm_2_stm32_struct));
			rx_step=0;
			rx_count=0;
		}
		break;

	default:
		rx_step=0;
		rx_count=0;
		break;
	}
}

void bsp_uart1_tx(void)
{
	stm32_2_apm.stm32_2_apm_t.header[0] = 0xAA;
	stm32_2_apm.stm32_2_apm_t.header[1] = 0xAC;
	stm32_2_apm.stm32_2_apm_t.len = sizeof(struct stm32_2_apm_struct);

    HAL_UART_Transmit(&huart1, stm32_2_apm.bits, sizeof(struct stm32_2_apm_struct), 0xff);
}

void bsp_uart1_get_feedback_wheel_speed(int16_t wheelleft, int16_t wheelright)
{
	stm32_2_apm.stm32_2_apm_t.wheel_left_int = wheelleft;
	stm32_2_apm.stm32_2_apm_t.wheel_right_int = wheelright;
}

void bsp_uart_get_target_wheel_speed(int16_t* wheelleft, int16_t* wheelright)
{
	*wheelleft = apm_2_stm32.apm_2_stm32_t.wheel_left_int;
	*wheelright = apm_2_stm32.apm_2_stm32_t.wheel_right_int;
}
