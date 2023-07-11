#include "bsp_uart.h"

#include "main.h"
#include "stdio.h"
#include "string.h"

float wheel_left_target;
float wheel_right_target;

uint8_t UART1_RX_BUF[RM_UART_MAX_LEN]; // 缓存数组
uint8_t UART1_RX_DATA;

union rmuart_t rmuart_rx;
union ardupilot_t ardupilot_tx;

///////////////////////////////////////////////////////////////////////////
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart ->Instance == USART1)
    {
    	bsp_uart_parse(UART1_RX_DATA);
		//等待下一次接收中断
		HAL_UART_Receive_IT(huart,&UART1_RX_DATA,1);
    }
}

///////////////////////////////////////////////////////////////////////////
void bsp_uart1_init(void)
{
	//开启接收中断
	HAL_UART_Receive_IT(&huart1, &UART1_RX_DATA, 1);
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
		if(data == sizeof(struct rmuart_struct))
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

		if(rx_count >= sizeof(struct rmuart_struct))
		{
			memcpy(rmuart_rx.bits, UART1_RX_BUF, sizeof(struct rmuart_struct));
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
    ardupilot_tx.ardupilot_s.header[0] = 0xAA;
    ardupilot_tx.ardupilot_s.header[1] = 0xAC;

    ardupilot_tx.ardupilot_s.len = sizeof(struct ardupilot_struct);

    HAL_UART_Transmit(&huart1, ardupilot_tx.bits, sizeof(struct ardupilot_struct), 0xff);
}

void bsp_uart1_set_wheel_speed(int16_t wheelleft, int16_t wheelright)
{
    ardupilot_tx.ardupilot_s.wheel_left_speed = wheelleft;
    ardupilot_tx.ardupilot_s.wheel_right_speed = wheelright;
}

void bsp_uart_get_wheel_speed(int16_t* wheelleft, int16_t* wheelright)
{
	*wheelleft = rmuart_rx.rmuart_s.wheel_left;
	*wheelright = rmuart_rx.rmuart_s.wheel_right;
}
