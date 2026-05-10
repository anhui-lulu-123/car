/*
 * Init.c
 *
 *  Created on: 2025��12��9��
 *      Author: 29526
 */
#include "Initial.h"
#include "zf_device_gnss.h"
#include "INS_Nav.h"
bool Initial_Flag=false;
//�ܳ�ʼ��
void SUM_Init(void)
{
     // TFT180 屏幕初始化
     tft180_init();
     tft180_clear();

     //  小刷板 串口通讯初始化
     small_driver_uart_init();

     //  陀螺仪初始化
     Attitude_Init();

    //  �����ʼ��
     Servo_Init();
     servo_input_duty = SERVO_MID;

     //ң������ʼ��
     Remote_Control_Init();

    //PID总初始化
     pidAllInit();

    // INS导航初始化
     INS_Nav_Init();

   // ʱ��Ƭ��ʼ��+��ʱ������
    pit_ms_init(CCU60_CH0,1);
    pit_ms_init(CCU60_CH1,5);

    // GPS 初始化暂时屏蔽：UART2 与遥控器复用，当前使用遥控器
    // gnss_init(TAU1201);

    // 按键初始化（内部上拉，按下=低电平），引脚顺序对齐 1and2 afew
    gpio_init(P20_6, GPI, GPIO_HIGH, GPI_PULL_UP);      // KEY1：启动/停止
    gpio_init(P20_7, GPI, GPIO_HIGH, GPI_PULL_UP);      // KEY2：惯导/状态重置
    gpio_init(P11_2, GPI, GPIO_HIGH, GPI_PULL_UP);      // KEY3：记录航点
    gpio_init(P11_3, GPI, GPIO_HIGH, GPI_PULL_UP);      // KEY4：全部重置

    // 拨码开关初始化（浮空输入）
    gpio_init(P33_11, GPI, GPIO_HIGH, GPI_FLOATING_IN); // SWITCH1：地图/数据切换

    Initial_Flag=true;
}
