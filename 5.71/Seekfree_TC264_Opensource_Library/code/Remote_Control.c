/*
 * Remote_Control.c
 *
 *  Created on: 2025��12��27��
 *      Author: 29526
 */



#include "Remote_Control.h"
#include "Control.h"

void Remote_Control_Init(void)
{
    uart_receiver_init();                   //遥控器接收机初始化
}

bool Remote_Connect_State = false;
float Last_uart_receiver_State_Conctrol_Channel_data;
float Remote_Dicretion_Targrt = 0;
#define RC_SPEED_MAX  1000    // ★ 在这里改最大速度(RPM)，中位自动是一半 ★

int16 Remote_Speed_Target = RC_SPEED_MAX / 2;  // 上电默认 400（中位值）

void Remote_Control(void)
{
    if(1 == uart_receiver.finsh_flag)
    {
        if(1 == uart_receiver.state)
        {
            Remote_Connect_State = true;

            // CH1 → 转向
            Remote_Dicretion_Targrt = -(float)(uart_receiver.channel[Direction_Conctrol_Channel] - Remote_Channel_Data_Middle)
                                      * Direction_Out_Max / 800.0f;

            // CH2 → 速度：192~1792 线性映射到 0~RC_SPEED_MAX，中位=RC_SPEED_MAX/2
            int16 ch2 = uart_receiver.channel[Speed_Conctrol_Channel];
            Remote_Speed_Target = (int16)((ch2 - Remote_Channel_Data_Min)
                                  * RC_SPEED_MAX / (float)(Remote_Channel_Data_Max - Remote_Channel_Data_Min));
        }
        else
        {
            Remote_Connect_State = false;
            Remote_Dicretion_Targrt = 0;
            Remote_Speed_Target = RC_SPEED_MAX / 2;
        }
        uart_receiver.finsh_flag = 0;
    }
}


void Remote_Control_Test(void)
{
    if(1 == uart_receiver.finsh_flag)                            // ֡��ɱ�־�ж�
       {
              if(1 == uart_receiver.state)                             // ң����ʧ��״̬�ж�
              {
                  printf("CH1-CH6 data: ");
                  for(int i = 0; i < 6; i++)
                  {
                      printf("%d ", uart_receiver.channel[i]);         // �������6��ͨ������
                  }
                  printf("\r\n");
              }
              else
              {
                  printf("Remote control has been disconnected.\r\n"); // �������ʧ����ʾ
              }
              uart_receiver.finsh_flag = 0;                            // ֡��ɱ�־��λ
       }
}

/**
 * @brief 使用CT-6A遥控器控制电机占空比 (0-50)
 * @note  通过遥控器通道值映射到电机占空比范围
 *        遥控器通道范围: 192(最小) - 992(中值) - 1792(最大)
 *        电机占空比范围: 0 - 50
 */
void Remote_Motor_Duty_Control(void)
{
    if(1 == uart_receiver.finsh_flag)                            // ֡��ɱ�־�ж�
    {
        if(1 == uart_receiver.state)                             // ң��������״̬�ж�
        {
            // ��ȡң�����ٶȿ���ͨ��ֵ (ͨ��2)
            int16 channel_value = uart_receiver.channel[Speed_Conctrol_Channel];

            // �޷����
            if(channel_value < Remote_Channel_Data_Min)
                channel_value = Remote_Channel_Data_Min;
            if(channel_value > Remote_Channel_Data_Max)
                channel_value = Remote_Channel_Data_Max;

            // ӳ��ң����ͨ��ֵ��ռ�ձ� (192~1792 -> 0~50)
            // ��ʽ: duty = (channel_value - 192) * 50 / (1792 - 192)
            int16 motor_duty = (int16)((channel_value - Remote_Channel_Data_Min) * 50 / (Remote_Channel_Data_Max - Remote_Channel_Data_Min));

            // �����������ձ� (���Ҳ���������)
            small_driver_set_duty(motor_duty, motor_duty);
        }
        else
        {
            // ң����ʧ����ֹͣ����
            small_driver_set_duty(0, 0);
        }

        uart_receiver.finsh_flag = 0;                            // ֡��ɱ�־��λ
    }
}
