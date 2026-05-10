/*
 * imu.c
 *
 *  Created on: 2023��2��5��
 *      Author: sorrymaker
 */

#include "Attitude.h"

IMU_DATA_Collect IMU_Data;


//һά�������˲���������ʼ��

KalmanFilter_Struct kf_acc[3]=
{
        { 0.02,0,0,0,0.001,0.543},
        { 0.02,0,0,0,0.001,0.543},
        { 0.02,0,0,0,0.001,0.543}
};

//IIR�˲���
IIR_cceff_Typedef Filter_Param_Gx;  ///< ������X���˲���
IIR_cceff_Typedef Filter_Param_Gy;  ///< ������Y���˲���
IIR_cceff_Typedef Filter_Param_Gz;  ///< ������Z���˲���

// �˲���ָ�룬���㺯������
IIR_cceff_Typedef *pFilter_Param_Gx = &Filter_Param_Gx;
IIR_cceff_Typedef *pFilter_Param_Gy = &Filter_Param_Gy;
IIR_cceff_Typedef *pFilter_Param_Gz = &Filter_Param_Gz;





/*
 *@brief   IIR�˲�����ʼ��
 *@param   void
 *@note
 *@retval  void
*/
void IMU_IIR_Filter_init(void)
{
    cal_irr_coeff(pFilter_Param_Gx,SAMPLE_COLLLECT_FREQUENCE,Gyro_cutoffFreq);
    cal_irr_coeff(pFilter_Param_Gy,SAMPLE_COLLLECT_FREQUENCE,Gyro_cutoffFreq);
    cal_irr_coeff(pFilter_Param_Gz,SAMPLE_COLLLECT_FREQUENCE,Gyro_cutoffFreq);

}

/*
 *@brief  ��ȡ��������ƫ���
 *@param   void
 *@note
 *@retval  void
*/
void Get_Gyro_Offest(void)
{
    IMU_Data.Gyro_Offest_calib_Success=0;
    static const uint16 N=4096;
    while(IMU_Data.Gyro_Offest_calib_Success==0)
    {

       //����D(X)=E(X^2)-(E(X)^2)�����Ż�
       double sumX[6]={0,0,0,0,0,0};
       double sumXX[6]={0,0,0,0,0,0};
       float gx,gy,gz,ax,ay,az;
        for(uint16 i=0;i<N;++i)
          {
//
                     imu660rb_get_gyro ();
                     imu660rb_get_acc ();
                     gx= imu660rb_gyro_transition(imu660rb_gyro_x);
                     gy= imu660rb_gyro_transition(imu660rb_gyro_y);
                     gz= imu660rb_gyro_transition(imu660rb_gyro_z);
                     ax=imu660rb_acc_transition(imu660rb_acc_x);
                     ay=imu660rb_acc_transition(imu660rb_acc_y);
                     az=imu660rb_acc_transition(imu660rb_acc_z);
                sumX[0]+= gx;
                sumX[1]+= gy;
                sumX[2]+= gz;
                sumX[3]+= ax;
                sumX[4]+= ay;
                sumX[5]+= az;
                sumXX[0]+=gx*gx;
                sumXX[1]+=gy*gy;
                sumXX[2]+=gz*gz;
                sumXX[3]+=ax*ax;
                sumXX[4]+=ay*ay;
                sumXX[5]+=az*az;
                system_delay_us(250);

            }
                 double EX[6]= {0,0,0,0,0,0};
                 double EXX[6]={0,0,0,0,0,0};
                 double DX[6]= {0,0,0,0,0,0};
                 for(uint8 i=0;i<6;++i)
               {
                  EX[i]=sumX[i]/N;
                  EXX[i]=sumXX[i]/N;
                  DX[i]=EXX[i]-EX[i]*EX[i];
               }
              //printf("\r\n Check_Imu_Position:%f,%f,%f,%f,%f,%f\r\n",DX[0],DX[1],DX[2],DX[3],DX[4],DX[5]);
               if((DX[0]<1e-1&&DX[1]<1.2e-1&&DX[2]<1e-1&&DX[3]<1e-4&&DX[4]<1e-4&&DX[5]<1e-4)&&(fabs(DX[0])>=1e-15&&fabs(DX[1])>=1e-15&&fabs(DX[2])>=1e-15&&fabs(DX[3])>=1e-15&&fabs(DX[4])>=1e-15&&fabs(DX[5])>=1e-15))
                    {

                            IMU_Data.Gyro_Offestx=(float)EX[0];
                            IMU_Data.Gyro_Offesty=(float)EX[1];
                            IMU_Data.Gyro_Offestz=(float)EX[2];
                            QEKF_INS.Zero_G=sqrtf(EX[3]*EX[3]+EX[4]*EX[4]+EX[5]*EX[5]);
                            QEKF_INS.Global_G_Bias[2] = QEKF_INS.Zero_G;
                            IMU_Data.Gyro_Offest_calib_Success= 1;
                    }

                  if(IMU_Data.Gyro_Offest_calib_Success)
                  {
                // printf("\r\n Gyro Offest data:  x=%f, y=%f, z=%f\r\n",  IMU_Data.Gyro_Offestx, IMU_Data.Gyro_Offesty, IMU_Data.Gyro_Offestz);
                                          break;
                    }

         }


}



/*
 *@brief  �������ܳ�ʼ��
 *@param   void
 *@note
 *@retval  void
*/
void Attitude_Init(void)
{

    imu660rb_init();//陀螺仪初始化
   Get_Gyro_Offest();//去零飘
    IMU_IIR_Filter_init();//IIR��ʼ��
    IMU_QuaternionEKF_Init(10, 5e-4, 1000000, 0.9996, 0.005f);//EKF������ʼ��

}

//��������̬����
void Attitude_Calculate(void)
{
    static uint8 counts_imu = 0;
    counts_imu ++;
    imu660rb_get_gyro();
    IMU_Data.Gyro_x= get_iir_output(pFilter_Param_Gx, imu660rb_gyro_transition((float)imu660rb_gyro_x)-IMU_Data.Gyro_Offestx)* 0.01745329f;//rad/s
    IMU_Data.Gyro_y= get_iir_output(pFilter_Param_Gy, imu660rb_gyro_transition((float)imu660rb_gyro_y)-IMU_Data.Gyro_Offesty)* 0.01745329f;
    IMU_Data.Gyro_z= get_iir_output(pFilter_Param_Gz, imu660rb_gyro_transition((float)imu660rb_gyro_z)-IMU_Data.Gyro_Offestz)* 0.01745329f;

    imu660rb_get_acc();

    IMU_Data.Accel_x=Com_Kalman_Filter(&kf_acc[0], imu660rb_acc_transition((float)imu660rb_acc_x));
    IMU_Data.Accel_y=Com_Kalman_Filter(&kf_acc[1], imu660rb_acc_transition((float)imu660rb_acc_y));
    IMU_Data.Accel_z=Com_Kalman_Filter(&kf_acc[2], imu660rb_acc_transition((float)imu660rb_acc_z));

 if (fabs( IMU_Data.Gyro_z) <= 0.008f)  IMU_Data.Gyro_z= 0.0f; // ��������
if(counts_imu %5==0)//5ms��̬����һ��
{
    IMU_QuaternionEKF_Update(IMU_Data.Gyro_x, IMU_Data.Gyro_y, IMU_Data.Gyro_z,IMU_Data.Accel_x,IMU_Data.Accel_y,IMU_Data.Accel_z);
     counts_imu =0;
}

}




// ���Ƕ�ֵ��һ���� [0, 360) ��Χ��
void angle_handle_360(float *angle)//�滮����ϵ�Ƕ�
{
  if(*angle>360){*angle-=360;}
  else if(*angle<0){*angle+=360;}
}


// ���Ƕ�ֵ��һ���� [-180, 180) ��Χ��
void angle_handle_180(float *angle)//�滮����ϵ�Ƕ�
{
  if(*angle>180){*angle-=360;}
  else if(*angle<-180){*angle+=360;}
}
