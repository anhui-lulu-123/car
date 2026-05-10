/*
 * Remote_Control.h
 *
 *  Created on: 2025��12��27��
 *      Author: 29526
 */

#ifndef CODE_REMOTE_CONTROL_H_
#define CODE_REMOTE_CONTROL_H_

#include "zf_common_headfile.h"
#define Remote_Channel_Data_Min    192
#define Remote_Channel_Data_Middle    992
#define Remote_Channel_Data_Max    1792


#define Speed_Each_Gear  50     //һ������Ӧ���ٶ�������ֵ
#define  Remote_Direction_Factor  15/800;
typedef enum {
    Direction_Conctrol_Channel,//���Ƴ�ģ�ķ���  192 -992- 1792
    Speed_Conctrol_Channel,//���Ƴ�ģ���ٶ�     192 -992 -1792
    State_Conctrol_Channel,//����״̬ 192/1792
    Remote_Channel_4,//����״̬  192/992/1792
    Remote_Channel_5,//����״̬ 192/1792
    Remote_Channel_6, //����״̬ 192/1792
}REMOTE_CHANNEL_NUM;
extern bool Remote_Connect_State;
extern float Remote_Dicretion_Targrt;
extern int16 Remote_Speed_Target;   // 遥控器速度目标(RPM)，范围 SPEED_MIN~SPEED_MAX
void Remote_Control_Init(void);
void Remote_Control(void);
void Remote_Control_Test(void);
void Remote_Motor_Duty_Control(void);
#endif /* CODE_REMOTE_CONTROL_H_ */
