#ifndef SMALL_DRIVER_UART_CONTROL_H_
#define SMALL_DRIVER_UART_CONTROL_H_

#include "zf_common_headfile.h"


#define SMALL_DRIVER_UART                       (UART_3        )

#define SMALL_DRIVER_BAUDRATE                   (460800        )

#define SMALL_DRIVER_RX                         (UART3_TX_P15_6)

#define SMALL_DRIVER_TX                         (UART3_RX_P15_7)

/* 单车外部速度指令范围：±2000；内部映射到小刷板占空比协议 ±10000 */
#define CYT2_S_MOTOR_MAX                        (2000)
#define CYT2_S_MOTOR_MIN                        (-2000)
#define CYT2_S_DUTY_SCALE                       (5)

typedef struct
{
    uint8 send_data_buffer[7];                  // ���ͻ�������

    uint8 receive_data_buffer[7];               // ���ջ�������

    uint8 receive_data_count;                   // ���ռ���

    uint8 sum_check_data;                       // У��λ

    int16 receive_left_speed_data;              // ���յ���������ٶ�����

    int16 receive_right_speed_data;             // ���յ����Ҳ����ٶ�����

}small_device_value_struct;

extern small_device_value_struct motor_value;



void uart_control_callback(void);                                   // ��ˢ���� ���ڽ��ջص�����

void small_driver_set_duty(int16 left_duty, int16 right_duty);      // 0x01 设置占空比 (-10000~10000)

void CYT2_S_motor_ctrl(int32 SPEED);                                // 单车小刷板：±2000 指令映射到左右通道占空比

void small_driver_get_speed(void);                                  // 0x02 获取速度信息

void small_driver_uart_init(void);                                  // 小刷板 串口通讯初始化

#endif
