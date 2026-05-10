/*
 * Encorder.h
 *
 *  Created on: 2025年12月26日
 *      Author: 29526
 */

#ifndef CODE_ENCORDER_H_
#define CODE_ENCORDER_H_

#include "zf_common_headfile.h"


#define ENCODER_QUADDEC                 (TIM2_ENCODER)                         // 正交编码器对应使用的编码器接口
#define ENCODER_QUADDEC_A               (TIM2_ENCODER_CH1_P33_7)               // A 相对应的引脚  TIM2_ENCODER_CH1_P33_7
#define ENCODER_QUADDEC_B               (TIM2_ENCODER_CH2_P33_6)               // B 相对应的引脚  TIM2_ENCODER_CH2_P33_6

#define ENCODER_COLLECT_PERIOD       5        // 数据采样间隔（单位：毫秒）
#define GEAR_RATIO                 15/60                //齿轮比
#define ENCODER_RESOLUTION           1024         // 编码器分辨率（每转脉冲数）
#define BACK_WhEEL_DIAMETER          0.07         //后轮直径 m

#define MOVE_AVERAGE_SIZE   10  //定义缓冲区大小

typedef struct {
    // 编码器数据
   int16 new_encoder;          // 新的编码器值（建议使用有符号类型）
   float Velocity;              // 速度（m/s）


   bool speed_get_flag;//读取磁编码器转速成功标志
   uint16 speed_get_fail_count;//计数
} WheelData;
extern WheelData Back_Wheel;


void Calculate_Encoder(void);
#endif /* CODE_ENCORDER_H_ */
