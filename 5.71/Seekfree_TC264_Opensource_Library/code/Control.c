/*
 * Control.c
 *
 *  Created on: 2025锟斤拷12锟斤拷19锟斤拷
 *      Author: 29526
 */

#include "Control.h"



float Balance_init_zero=-0.7f;  // 向右补偿左倾偏置，测试中

int8   car_mode_switch_num=1;  // 0=鎱㈤�熷崟鐜疨ID(璋冭瘯), 1=蹇�熷弻鐜疨ID

uint16 servo_input_duty;

int16 Bike_State = Stop;  // 对齐 1and2：上电停止，按 KEY1 切换启动/停止

float Direction_Out_Max=8.0f;  // 转向环最大偏移量（单位：度）
void Balance_Control(void)//平锟斤拷锟斤拷锟�
{
    if(car_mode_switch_num==1)//锟斤拷锟斤拷
    {
        static uint8 counts_1 = 0;
       counts_1++;

      {
               if (counts_1 % 20 == 0)//20ms
                 {
                      PID_Calculate(&Balance_AnglePid,Balance_init_zero +Direction_Pid.pos_out, QEKF_INS.Pitch);//锟角度伙拷PD
                      counts_1=0;
                  }
               //1ms
                PID_Calculate(&Balance_angular_velocityPid,0+Balance_AnglePid.pos_out,-QEKF_INS.Gyro[1]*57.29578f);// Gyro[1]=Pitch角速度，对应左右倾轴//锟斤拷锟劫度伙拷PD


                servo_input_duty=SERVO_MID-(int16)Balance_angular_velocityPid.pos_out;

      servo_input_duty=LIMIT(servo_input_duty,GetServoDuty(-SERVO_MAX_ANGLE), GetServoDuty(SERVO_MAX_ANGLE));//锟睫凤拷
      }

      if (abs(QEKF_INS.Pitch) > 35)//锟斤拷锟斤拷锟斤拷斜锟斤拷锟斤拷---停锟斤拷
      {
             Bike_State = Stop;
             return;
      }
      if ( Bike_State == Stop && abs(QEKF_INS.Pitch-Balance_init_zero) < 2.0f)//锟斤拷锟斤拷锟斤拷锟�--锟斤拷锟斤拷
      {

             pidClear(&Balance_angular_velocityPid);
             pidClear(&Balance_AnglePid);
             Bike_State = Null;
             counts_1 = 0;
       }
       if (Bike_State!=Stop)//停锟斤拷锟斤拷锟絧wm锟斤拷锟斤拷锟斤拷锟�--锟睫凤拷锟角讹拷
       {
           pwm_set_duty(SERVO_PIN, servo_input_duty);
       }
    }
    else if(car_mode_switch_num==0)//锟斤拷锟斤拷
    {
        static uint8 counts_0 = 0;
              counts_0++;

              if (counts_0 % 5 == 0)//5ms
                {
                    PID_Calculate(&Slow_Speed_Balance_AnglePid,Balance_init_zero + Direction_Pid.pos_out, QEKF_INS.Pitch);//锟角度伙拷PD
                    counts_0=0;
                }

              servo_input_duty=SERVO_MID+(int16)Slow_Speed_Balance_AnglePid.pos_out;  // 左倾→Pitch正→误差负→pos_out负→duty减小→舵机左转(修正)

             servo_input_duty=LIMIT(servo_input_duty,GetServoDuty(-SLOW_SERVO_MAX_ANGLE), GetServoDuty(SLOW_SERVO_MAX_ANGLE));//锟睫凤拷

              if (abs(QEKF_INS.Pitch) >35)//锟斤拷锟斤拷锟斤拷斜锟斤拷锟斤拷---停锟斤拷
              {
                     Bike_State = Stop;
                     return;
              }
              if ( Bike_State == Stop && abs(QEKF_INS.Pitch-Balance_init_zero) <2.0f)//锟斤拷锟斤拷锟斤拷锟�--锟斤拷锟斤拷
              {
                     pidClear(&Slow_Speed_Balance_AnglePid);
                     Bike_State = Null;
                     counts_0 = 0;
               }

               if (Bike_State!=Stop)//停锟斤拷锟斤拷锟絧wm锟斤拷锟斤拷锟斤拷锟�--锟睫凤拷锟角讹拷
               {
                   pwm_set_duty(SERVO_PIN, servo_input_duty);
               }


    }

}


// 后轮驱动力指令（与 1and2 一致经 CYT2_S_motor_ctrl 映射为左通道占空比，约 ±2000 对应满行程）
int16 target_speed = 1000;

void Back_Motor_Control(void)
{
    switch (Bike_State)
    {
        case Null:
            CYT2_S_motor_ctrl(0);
            break;
        case Run:
            CYT2_S_motor_ctrl((int32)target_speed);
            break;
        case Stop:
            CYT2_S_motor_ctrl(0);
            break;
        default:
            break;
    }
}
