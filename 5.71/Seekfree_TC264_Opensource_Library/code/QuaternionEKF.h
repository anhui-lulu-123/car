/*
 * QuaternionEKF.h
 *
 *  Created on: 2025年12月1日
 *      Author: 29526
 */

#ifndef CODE_QUATERNIONEKF_H_
#define CODE_QUATERNIONEKF_H_

#include "Kalman_Filter.h"



/* boolean type definitions */
#ifndef TRUE
#define TRUE 1 /**< boolean true  */
#endif

#ifndef FALSE
#define FALSE 0 /**< boolean fails */
#endif

typedef struct
{
    uint8_t Initialized;
    KalmanFilter_t IMU_QuaternionEKF;
    uint8_t ConvergeFlag;
    uint8_t StableFlag;
    uint64_t ErrorCount;
    uint64_t UpdateCount;

    float q[4];        // 四元数估计值

    float GyroBias[3]; // 陀螺仪零偏估计值

    float Gyro[3];
    float Accel[3];
    float Global_Acc[3];//世界坐标下的加速度
    float Last_Global_Acc[3];//世界坐标下的加速度
    float Body_G_Bias[3]; //载体重力偏差
    float Global_G_Bias[3]; //载体重力偏差

    float Body_Acc[3];      //去掉重力载体坐标下的加速度
    float Zero_G;       //Z轴重力

    float OrientationCosine[3];


    float gyro_norm;
    float accl_norm;
    float AdaptiveGainScale;

    float Roll;
    float Pitch;
    float Yaw;

    float Lpf_Roll;
    float Lpf_k;
    float Last_Roll;
    float Roll_Cut_Frequnce;
    float Last_Roll_Cut_Frequnce;
    float Yaw_Bias;

    float YawTotalAngle;

    float Q1; // 四元数更新过程噪声
    float Q2; // 陀螺仪零偏过程噪声
    float R;  // 加速度计量测噪声

    float dt; // 姿态更新周期
    mat ChiSquare;
    float ChiSquare_Data[1];      // 卡方检验检测函数
    float ChiSquareTestThreshold; // 卡方检验阈值
    float lambda;                 // 渐消因子


} QEKF_INS_t;

extern QEKF_INS_t QEKF_INS;
extern float chiSquare;
extern float ChiSquareTestThreshold;

void IMU_QuaternionEKF_Init(float process_noise1, float process_noise2, float measure_noise, float lambda, float dt);
void IMU_QuaternionEKF_Update(float gx, float gy, float gz, float ax, float ay, float az);
void IMU_QuaternionEKF_Reset(void);
extern float invSqrt(float x);
void Quaternion_Rotate_Vector(const float q[4],float vector[3],uint8 rev);

#endif /* CODE_QUATERNIONEKF_H_ */
