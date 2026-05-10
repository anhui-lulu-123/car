/*
 * Servo.h
 *
 *  Created on: 2025年12月28日
 *      Author: 29526
 */

#ifndef CODE_SERVO_H_
#define CODE_SERVO_H_
#include "zf_common_headfile.h"

/* 舵机频率 */
#define SERVO_FREQ          300                                  // 工作频率(50-300Hz)

/* 舵机中位占空比(0度位置) */
#define SERVO_MID           GetServoDuty(0)                     // 舵机中位占空比

/* 舵机旋转角度(物理角度) */
#define SERVO_MAX_ANGLE 45                                             // 物理极限角度 (对应 duty 3620~6220)

#define SLOW_SERVO_MAX_ANGLE     60                                      // 慢速角度

/* 舵机旋转角度对应占空比变化量 */
#define SERVO_DUTY_MAX      (GetServoDuty(SERVO_MAX_ANGLE) - SERVO_MID)      // 舵机旋转角度对应占空比

#define SLOW_SERVO_DUTY_MAX      (GetServoDuty( SLOW_SERVO_MAX_ANGLE) - SERVO_MID)      // 舵机旋转角度对应占空比

/* 舵机PWM输出引脚 */
#define SERVO_PIN            (ATOM1_CH1_P33_9)              // 舵机PWM输出引脚


extern float  global_servo_calibration;     // 舵机校准偏移量

uint16 GetServoDuty(float angle);

float GetServoAngle(uint16 duty);

void Servo_Init(void);


#endif /* CODE_SERVO_H_ */
