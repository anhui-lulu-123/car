/*
 * PID.h
 *
 *  Created on: 2025年12月11日
 *      Author: 29526
 */

#ifndef CODE_PID_H_
#define CODE_PID_H_

#include "zf_common_headfile.h"

/* ========= 宏定义 ========= */

/* 绝对值宏：返回x的绝对值 */
#define ABS(x) (((x) > 0) ? (x) : -(x))

/* 圆周率π的近似值 */
#define pi 3.14159265358979






/* 限幅宏：将x限制在[low, high]范围内 */
#define Limitation(x, low, high) ((x) < (low) ? (low) : ((x) > (high) ? (high) : (x)))



/* ========= 枚举定义 ========= */

/* 时间索引枚举：用于数组下标，表示不同时刻的数据 */
enum
{
    LLAST = 0,   // 前前次（k-2时刻）
    LAST  = 1,   // 前次（k-1时刻）
    NOW   = 2,   // 当前（k时刻）
    /* PID模式枚举 */
    POSITION_PID,  // 位置式PID
    DELTA_PID,     // 增量式PID
};

/* 外部变量声明（可能在moto.c中定义） */
extern uint8_t motionflag;  // 运动标志位

/**
 * @brief PID控制器结构体
 * 包含PID控制所需的所有参数和状态变量
 *
 * 成员变量说明：
 * - Kp, Ki, Kd: PID三个基本参数
 * - target[3]: 目标值数组（NOW/LAST/LLAST三个时刻）
 * - feedback[3]: 反馈值数组
 * - err[3]: 误差值数组
 * - pout, iout, dout: 三个分量的输出
 * - pos_out/last_pos_out: 位置式PID的输出/上次输出
 * - delta_u/delta_out/last_delta_out: 增量式PID的增量/输出/上次输出
 * - max_err: 最大允许误差（超过则输出0）
 * - deadband: 死区范围（误差小于此值输出0）
 * - pid_mode: PID模式（位置式/增量式）
 * - MaxOutput: 最大输出限制
 * - IntegralLimit: 积分限幅
 * - smoothTargetKp/sport_kp_step/is_sporting: 平滑Kp切换相关参数
 */
typedef struct _PID_Typedef
{
    /* PID基本参数 */
    float  Kp;  // 比例系数
    float  Ki;  // 积分系数
    float  Kd;  // 微分系数

    /* 数据数组（三个时刻的历史数据） */
    float target[3];    // 目标值：0=LLAST, 1=LAST, 2=NOW
    float feedback[3];  // 反馈值
    float err[3];       // 误差值

    /* PID各项输出 */
    float pout;  // 比例项输出
    float iout;  // 积分项输出
    float dout;  // 微分项输出

    /* 位置式PID相关 */
    float pos_out;       // 当前输出值
    float last_pos_out;  // 上次输出值
    float pos_out0;      // 位置式PID最小输出值（可能用于死区补偿）

    /* 增量式PID相关 */
    float delta_u;           // 控制增量
    float last_delta_out;    // 上次增量输出
    float delta_out;         // 当前增量输出

    /* 限制和保护参数 */
    float max_err;    // 最大允许误差
    float deadband;   // 死区范围

    /* 控制模式 */
    uint32_t pid_mode;  // PID模式：POSITION_PID或DELTA_PID

    /* 输出限制 */
    float MaxOutput;       // 输出最大值限制
    float IntegralLimit;   // 积分项最大值限制



    float Set;



} PID_TypeDef;

/* ========= 全局PID控制器实例声明 ========= */

/* 直立环角度PID */
extern PID_TypeDef Balance_AnglePid;
/* 慢速直立环角度PID */
extern PID_TypeDef Slow_Speed_Balance_AnglePid;
/* 直立环动态角度PID */
extern PID_TypeDef Dynamic_Balance_AnglePid;
/* 直立环动态角速度PID*/
extern PID_TypeDef Dynamic_Balance_angular_velocityPid;
/* 直立环角速度PID*/
extern PID_TypeDef Balance_angular_velocityPid;
/* 后轮速度PID：控制后轮电机的转速 */
extern PID_TypeDef BackSpeed_Pid;
/* 方向舵PID：控制舵机的转向角度 */
extern PID_TypeDef Direction_Pid;

extern  PID_TypeDef LQR_Balance_Pid;
/* 预设标志位声明 */
extern bool preset1, preset2, preset3;

/* ========= 函数声明 ========= */

/**
 * @brief PID控制器初始化
 * @param pid            PID控制器指针
 * @param mode           PID模式
 * @param maxout         最大输出限制
 * @param intergral_limit 积分限制
 * @param kp             比例系数
 * @param ki             积分系数
 * @param kd             微分系数
 *
 * 功能：初始化PID控制器所有参数
 */
void PID_Init(PID_TypeDef* pid, uint32_t mode,float maxout, float intergral_limit, float kp,float ki,float kd);

/**
 * @brief 快速重置PID参数
 * @param pid PID控制器指针
 * @param kp  比例系数
 * @param ki  积分系数
 * @param kd  微分系数
 *
 * 功能：仅修改PID三个基本系数
 */
void PID_Reset(PID_TypeDef *pid, float kp, float ki, float kd);

/**
 * @brief PID计算函数
 * @param pid      PID控制器指针
 * @param target   目标值
 * @param feedback 反馈值
 * @return float   PID输出值
 *
 * 功能：计算PID输出，支持位置式和增量式两种模式
 */
float PID_Calculate(PID_TypeDef *pid, float target, float feedback);

/**
 * @brief 所有PID控制器初始化
 *
 * 功能：初始化系统中所有需要用到的PID控制器
 */
void pidAllInit(void);

/**
 * @brief 清除PID控制器状态
 * @param pid PID控制器指针
 *
 * 功能：将PID所有中间变量清零
 */
void pidClear(PID_TypeDef *pid);




#endif /* CODE_PID_H_ */
