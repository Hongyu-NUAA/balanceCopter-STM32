#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include "usart.h"

#define PACKED __packed
#define MOTOR_MAX_SPEED 1000 // 此处为最大的电机转速

#define RM_UART_MAX_LEN    50
#define RM_UART_STRUCT_LEN 15

extern uint8_t UART1_RX_BUF[RM_UART_MAX_LEN]; // 缓存数组
extern uint8_t UART1_RX_LEN;                  // 缓存数组长度



struct PACKED apm_2_stm32_struct {
    uint8_t header[2];
    uint8_t len;
    int16_t wheel_left_int;
    int16_t wheel_right_int;
};
union apm_2_stm32_union {
    struct apm_2_stm32_struct apm_2_stm32_t;
    uint8_t bits[sizeof(struct apm_2_stm32_struct)];
};

struct PACKED stm32_2_apm_struct {
    uint8_t header[2];
    uint8_t len;
    int16_t wheel_left_int;
    int16_t wheel_right_int;
};
union stm32_2_apm_union {
    struct stm32_2_apm_struct stm32_2_apm_t;

    uint8_t bits[sizeof(struct stm32_2_apm_struct)];
};


void bsp_uart_parse(uint8_t data);

void bsp_uart1_get_feedback_wheel_speed(int16_t wheelleft, int16_t wheelright);

void bsp_uart_get_target_wheel_speed(int16_t* wheelleft, int16_t* wheelright);

void bsp_uart1_init(void);

void bsp_uart1_tx(void);

void bsp_uart1_handler(void);

#endif
