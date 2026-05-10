/*
 * Servo.c
 *
 *  Created on: 2025锟斤拷12锟斤拷28锟斤拷
 *      Author: 29526
 */


#include "Servo.h"

float global_servo_calibration = 1.51f;// 鑸垫満涓�兼牎鍑嗭紝瀹炴祴鏍″噯鍊�


void Servo_Init(void)
{
    pwm_init(SERVO_PIN,SERVO_FREQ, 0);
    pwm_set_duty(SERVO_PIN,SERVO_MID );
}
/*
 * 锟斤拷锟斤拷转锟斤拷嵌燃锟斤拷锟斤拷锟絇WM占锟秸憋拷
 * 锟角度碉拷占锟秸比碉拷转锟斤拷锟斤拷式锟斤拷
 *         10000 * (1.5 + x / 90)
 * duty = ------------------------   (-90 <= x <= 90)
 *              1000 / freq
 */
uint16 GetServoDuty(float angle)//锟斤拷锟斤拷值为锟角讹拷
{
    return (uint16)((float)(PWM_DUTY_MAX * (global_servo_calibration + (float)angle / 90.0)) / (1000.0 / (float)SERVO_FREQ));
}


float GetServoAngle(uint16 duty)
{
    // 锟斤拷锟斤拷占锟秸比讹拷应锟侥角讹拷
    return (float)(90.0f * ((uint16)duty * (1000.0f / (float)SERVO_FREQ) / (float)PWM_DUTY_MAX - global_servo_calibration));

}
