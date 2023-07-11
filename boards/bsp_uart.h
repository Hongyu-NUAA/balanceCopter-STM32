#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include "usart.h"

#define MOTOR_MAX_SPEED 1000 // 此处为最大的电机转速

#define RM_UART_MAX_LEN    50
#define RM_UART_STRUCT_LEN 15

extern uint8_t UART1_RX_BUF[RM_UART_MAX_LEN]; // 缓存数组
extern uint8_t UART1_RX_LEN;                  // 缓存数组长度

struct __packed rmuart_struct {
    uint8_t header[2];
    uint8_t len;
    int16_t wheel_left;
    int16_t wheel_right;
};
union rmuart_t {
    struct rmuart_struct rmuart_s;
    uint8_t bits[sizeof(struct rmuart_struct)];
};

struct __packed ardupilot_struct {
    uint8_t header[2];
    uint8_t len;
    int16_t wheel_left_speed;
    int16_t wheel_right_speed;
};
union ardupilot_t {
    struct ardupilot_struct ardupilot_s;

    uint8_t bits[sizeof(struct ardupilot_struct)];
};

void bsp_uart_parse(uint8_t data);

void bsp_uart1_set_wheel_speed(int16_t wheelleft, int16_t wheelright);

void bsp_uart_get_wheel_speed(int16_t* wheelleft, int16_t* wheelright);

void bsp_uart1_init(void);

void bsp_uart1_tx(void);


#endif
