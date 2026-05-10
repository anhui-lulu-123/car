/*
 * Encorder.c
 *
 *  Created on: 2025年12月26日
 *      Author: 29526
 */


#include "Encorder.h"

WheelData Back_Wheel;

//-------------------------------------------------------------------------------------------------------------------
//  @brief      计算编码器值和速度
//  @return     void
//  @note       应该在固定的时间间隔调用（如5ms）
//-------------------------------------------------------------------------------------------------------------------
void Calculate_Encoder(void)
{

    if(Back_Wheel.speed_get_flag==false)
    {
        Back_Wheel.speed_get_fail_count++;
        if( Back_Wheel.speed_get_fail_count>=400)//2s读取一次
        {
            Back_Wheel.speed_get_fail_count=0;
            small_driver_get_speed();                                                                   // 获取实时速度数据
        }
        if(abs(motor_value.receive_left_speed_data)>1||abs(motor_value.receive_right_speed_data)>1)
        {
            Back_Wheel.speed_get_flag=true;
        }
    }


        Back_Wheel.new_encoder =motor_value.receive_right_speed_data;//磁编码器每分钟转速

        Back_Wheel.Velocity=(float) Back_Wheel.new_encoder/60.0*(BACK_WhEEL_DIAMETER)*PI ;//转化为m/s

}






