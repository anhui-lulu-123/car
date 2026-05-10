/*
 * Filter.h
 *
 *  Created on: 2025年12月4日
 *      Author: 29526
 */

#ifndef CODE_FILTER_H_
#define CODE_FILTER_H_

#include "zf_common_headfile.h"


/* 圆周率π的浮点表示 */
#define M_PI_F 3.141592653589793f

/* 一阶低通滤波器宏定义（简化调用） */
#define LPF_1_(hz, t, in, out) ((out) += (1 / (1 + 1 / ((hz) * 6.28f * (t)))) * ((in) - (out)))

/*
 * @brief IIR滤波器系数与状态结构体
 * @note 采用直接II型结构（正准型）
 */
typedef struct
{
    float fc;     ///< 截止频率，用于判断是否启用滤波
    float a1;     ///< 分母系数 a1
    float a2;     ///< 分母系数 a2
    float b0;     ///< 分子系数 b0
    float b1;     ///< 分子系数 b1
    float b2;     ///< 分子系数 b2
    float y_1;    ///< 延迟单元1：存储 y[n-1]
    float y_2;    ///< 延迟单元2：存储 y[n-2]
} IIR_cceff_Typedef;

/*
 * @brief 一维卡尔曼滤波器
 * @note 用于对一维信号进行最优估计，适用于传感器数据滤波等场景
 */
typedef struct
{
    float LastP;    // 上一时刻的估计误差协方差（a posteriori estimate error covariance）
    float NowP;     // 当前时刻的估计误差协方差（a priori estimate error covariance）
    float out;      // 卡尔曼滤波器的输出值，即最优估计状态

    float Kg;       // 卡尔曼增益（Kalman Gain），决定信任预测值还是测量值的权重
    float Q;        // 过程噪声协方差（Process noise covariance）
                    // 表示系统模型的不确定性，值越大滤波器对新测量值响应越快
    float R;        // 测量噪声协方差（Measurement noise covariance）
                    // 表示传感器测量的不确定性，值越大滤波器对测量值信任度越低
}KalmanFilter_Struct;

/* ========= 巴特沃斯滤波器结构体 ========= */

/**
 * @brief 巴特沃斯滤波器缓冲区数据结构
 * 用于存储滤波器的输入/输出历史数据
 */
typedef struct
{
    float Input_Butter[3];   // 输入历史：x[n-2], x[n-1], x[n]
    float Output_Butter[3];  // 输出历史：y[n-2], y[n-1], y[n]
} Butter_BufferData;

/**
 * @brief 巴特沃斯滤波器参数结构
 * 存储2阶IIR滤波器的系数
 */
typedef struct
{
    float a[3];  // 分母系数：a[0], a[1], a[2]，a[0]通常为1
    float b[3];  // 分子系数：b[0], b[1], b[2]
} Butter_Parameter;

/* 外部变量声明（在filter.c中定义） */
extern Butter_BufferData Butter_5HZ_Buffer_Acce[3];  // 5Hz截止频率的加速度计缓冲区
extern float Acce_Control[3], Acce_Control_Feedback[3];  // 加速度控制和反馈数组

/* ========= 函数声明 ========= */

/**
 * @brief 计算一阶低通滤波器的alpha参数
 * @param cutoff_frequency 截止频率（Hz）
 * @param time_step 时间步长（秒）
 * @return float alpha参数
 */
float set_lpf_alpha(int16 cutoff_frequency, float time_step);

/**
 * @brief 加速度控制滤波器（函数体可能在别处定义）
 */
void Acce_Control_Filter(void);

/* ========= 其他滤波器结构体 ========= */

/**
 * @brief 固定积分滤波器状态结构
 * 用于带修正的积分滤波器
 */
typedef struct
{
    float in_est_d;   // 估计的输入变化率
    float in_obs;     // 观测的输入值
    float fix_kp;     // 比例修正系数
    float e_limit;    // 误差限制值
    float e;          // 当前误差
    float out;        // 滤波器输出
} _fix_inte_filter_st;

/**
 * @brief 固定积分滤波器
 * @param dT 时间步长
 * @param data 滤波器状态指针
 */
void fix_inte_filter(float dT, _fix_inte_filter_st *data);

/**
 * @brief 限幅滤波器状态结构
 */
typedef struct
{
    float lpf_1;  // 一阶低通滤波后的值
    float out;    // 最终输出值
} _lf_t;

/**
 * @brief 限幅滤波器
 * @param T 时间常数
 * @param hz 截止频率
 * @param data 滤波器状态指针
 * @param in 输入值
 */
void limit_filter(float T, float hz, _lf_t *data, float in);

/**
 * @brief 最速下降法滤波器状态结构
 * 用于最速下降法滤波
 */
typedef struct
{
    uint8 cnt;                // 缓冲区计数器
    int32 lst_pow_sum;        // 上一次的平方和
    int32 now_out;            // 当前输出值
    int32 lst_out;            // 上一次输出值
    int32 now_velocity_xdt;   // 当前速度估计
} _steepest_st;



float Com_Kalman_Filter(KalmanFilter_Struct *kf,float input);
void steepest_descend(int32 arr[], uint8 len, _steepest_st *steepest, uint8 step_num, int32 in);
void Moving_Average(float moavarray[], uint16 len, uint16 *fil_cnt, float in, float *out);
float LowPassFilter1st(const float* data,const float* newData,const float coff);
float Sliding_weighted_filter(float* array ,uint8_t array_num,float data);
void cal_irr_coeff(IIR_cceff_Typedef* coeff, float fs, float fc);
float get_iir_output(IIR_cceff_Typedef* coeff, float data);

#endif /* CODE_FILTER_H_ */
