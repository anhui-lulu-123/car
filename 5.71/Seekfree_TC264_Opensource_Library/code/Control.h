 /*
 * Control.h
 *
 *  Created on: 2025年12月19日
 *      Author: 29526
 */

#ifndef CODE_CONTROL_H_
#define CODE_CONTROL_H_

#include "zf_common_headfile.h"

typedef enum                     //行驶方式的控制
{
   Null                    ,//待机状态
   Remote                  ,//遥控器状态
   Run                     ,//发车
   Stop                    ,//停车状态

}BIKE_STATE;                     //车体运行状态

extern int16 Bike_State;
extern int8  car_mode_switch_num;
extern float Balance_init_zero;
extern uint16 servo_input_duty;
extern float Direction_Out_Max;
extern int16 target_speed;  // 后轮驱动力指令，经 CYT2_S_motor_ctrl→小刷板占空比（同 1and2）

void Balance_Control(void);
void Back_Motor_Control(void);
#endif /* CODE_CONTROL_H_ */
