/*
 * imu.h
 *
 *  Created on: 2023年2月5日
 *      Author: sorrymaker
 */

#ifndef CODE_ATTITUDE_H_
#define CODE_ATTITUDE_H_


#include "zf_common_headfile.h"

#define SAMPLE_COLLLECT_FREQUENCE 1000
#define Gyro_cutoffFreq 12//角速度


typedef struct
{
     float  Gyro_x,Gyro_y,Gyro_z;//角速度

     float  Gyro_Offestx,Gyro_Offesty,Gyro_Offestz;//角速度零偏值

     char    Gyro_Offest_calib_Success;//陀螺仪零偏校准成功标志位


     float  Accel_x,Accel_y,Accel_z;//加速度
     float  Last_Accel_x,Last_Accel_y,Last_Accel_z;//加速度
     float  Accel_Param_Scalex,Accel_Param_Scaley,Accel_Param_Scalez;//校准刻度
     float  Accel_Param_Biasx,Accel_Param_Biasy,Accel_Param_Biasz;//校准偏差


     float  Mag_x,Mag_y,Mag_z;//磁力计数据
     float  Mag_Param_Scalex,Mag_Param_Scaley,Mag_Param_Scalez;//XYZ轴偏移量
     float  Mag_Param_Biasx,Mag_Param_Biasy,Mag_Param_Biasz;//XYZ缩放因子


}IMU_DATA_Collect;


extern  IMU_DATA_Collect IMU_Data;
void IMU_IIR_Filter_init(void);
void Get_Gyro_Offest(void);
void angle_handle_360(float *angle);
void angle_handle_180(float *angle);
void Attitude_Init(void);
void Attitude_Calculate(void);
#endif /* CODE_ATTITUDE_H_ */
