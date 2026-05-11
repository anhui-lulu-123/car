/*
 * PID.c
 *
 *  Created on: 2025锟斤拷12锟斤拷11锟斤拷
 *      Author: 29526
 */

#include "PID.h"

#define HIGH_SPEED_TARGET (-80)   // 快速行驶
#define SPEED_KP          0.3f
#define SPEED_KI          0.05f
#define LOW_SPEED_TARGET  (-30)   // 慢速精确
#define LOW_SPEED_KP      0.2f
#define LOW_SPEED_KI      0.02f

/* 预锟斤拷锟街疚伙拷锟斤拷锟斤拷锟斤拷锟斤拷诓锟酵拷锟絇ID锟斤拷锟斤拷预锟借） */
bool preset1 = false, preset2 = false, preset3 = false;
/* ========= PID锟斤拷锟斤拷锟斤拷实锟斤拷锟斤拷锟斤拷 ========= */

/* 直锟斤拷锟斤拷锟角讹拷PID */
PID_TypeDef Balance_AnglePid;

/* 锟斤拷锟斤拷直锟斤拷锟斤拷锟角讹拷PID */
PID_TypeDef Slow_Speed_Balance_AnglePid;

PID_TypeDef Dynamic_Balance_AnglePid;//锟斤拷态

/* 直锟斤拷锟斤拷锟斤拷锟劫讹拷PID*/
PID_TypeDef Balance_angular_velocityPid;

PID_TypeDef Dynamic_Balance_angular_velocityPid;//锟斤拷态
/* 锟斤拷锟斤拷锟劫讹拷PID锟斤拷锟斤拷锟狡猴拷锟街碉拷锟斤拷锟阶拷锟� */
PID_TypeDef BackSpeed_Pid;
/* 锟斤拷锟斤拷锟絇ID锟斤拷锟斤拷锟狡讹拷锟斤拷锟阶拷锟角讹拷 */
PID_TypeDef Direction_Pid;
/* LQR锟角度伙拷PID */
PID_TypeDef LQR_Balance_Pid;


/**
 * @brief 锟斤拷锟斤拷PID锟斤拷锟斤拷
 */
void PID_Reset(PID_TypeDef *pid, float kp, float ki, float kd)
{
    pid->Kp = kp;  // 锟斤拷锟矫憋拷锟斤拷系锟斤拷
    pid->Ki = ki;  // 锟斤拷锟矫伙拷锟斤拷系锟斤拷
    pid->Kd = kd;  // 锟斤拷锟斤拷微锟斤拷系锟斤拷
}

/**
 * @brief PID锟斤拷锟斤拷锟斤拷锟斤拷始锟斤拷锟斤拷锟斤拷
 * @param pid            PID锟斤拷锟斤拷锟斤拷锟结构锟斤拷指锟斤拷
 * @param mode           PID模式锟斤拷POSITION_PID(位锟斤拷式)锟斤拷DELTA_PID(锟斤拷锟斤拷式)
 * @param maxout         锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷值
 * @param intergral_limit 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷值锟斤拷锟斤拷锟斤拷锟街憋拷锟酵ｏ拷
 * @param kp             锟斤拷锟斤拷系锟斤拷
 * @param ki             锟斤拷锟斤拷系锟斤拷
 * @param kd             微锟斤拷系锟斤拷
 */
void PID_Init(PID_TypeDef* pid,       uint32_t mode,         float maxout,           float intergral_limit, float kp,              float ki,               float kd)
{
    pid->IntegralLimit = intergral_limit;  // 锟斤拷锟矫伙拷锟斤拷锟斤拷锟斤拷
    pid->MaxOutput = maxout;               // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟�
    pid->pid_mode = mode;                  // 锟斤拷锟斤拷PID模式

    pid->target[0] = 0;  // 锟斤拷锟斤拷目锟斤拷值锟斤拷NOW位锟矫ｏ拷
    pid->Kp = kp;        // 锟斤拷锟矫憋拷锟斤拷系锟斤拷
    pid->Ki = ki;        // 锟斤拷锟矫伙拷锟斤拷系锟斤拷
    pid->Kd = kd;        // 锟斤拷锟斤拷微锟斤拷系锟斤拷

}

/**
 * @brief 锟斤拷锟斤拷值锟斤拷锟狡猴拷锟斤拷锟斤拷锟斤拷态锟斤拷锟斤拷锟斤拷只锟节碉拷前锟侥硷拷使锟矫ｏ拷
 */
static void abs_limit(float *num, float Limit)
{
    if (*num > Limit)          // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
    {
        *num = Limit;          // 锟斤拷锟狡碉拷锟斤拷锟斤拷锟斤拷
    }
    else if (*num < -Limit)    // 锟斤拷锟节革拷锟斤拷锟斤拷
    {
        *num = -Limit;         // 锟斤拷锟狡碉拷锟斤拷锟斤拷锟斤拷
    }
}

/**
 * @brief PID锟斤拷锟斤拷锟斤拷暮锟斤拷锟�
 * @param pid      PID锟斤拷锟斤拷锟斤拷锟结构锟斤拷指锟斤拷
 * @param target   目锟斤拷值锟斤拷锟借定值锟斤拷
 * @param feedback 锟斤拷锟斤拷值锟斤拷锟斤拷锟斤拷值锟斤拷
 * @return float   PID锟斤拷锟斤拷锟斤拷锟街�
 * 锟斤拷锟杰ｏ拷锟斤拷锟捷碉拷前锟斤拷锟斤拷锟斤拷PID锟斤拷锟�
 * 1. 位锟斤拷式PID锟斤拷POSITION_PID 2. 锟斤拷锟斤拷式PID锟斤拷DELTA_PID锟斤拷
 */
float PID_Calculate(PID_TypeDef *pid, float target, float feedback)
{
    /* 锟斤拷锟铰碉拷前时锟教碉拷锟斤拷锟斤拷 */
    pid->feedback[NOW] = feedback;  // 锟斤拷前锟斤拷锟斤拷值
    pid->target[NOW] = target;      // 锟斤拷前目锟斤拷值
    pid->err[NOW] = target - feedback;  // 锟斤拷前锟斤拷锟� = 目锟斤拷 - 锟斤拷锟斤拷

    /* 锟斤拷锟斤拷锟斤拷锟角否超筹拷锟斤拷锟斤拷锟斤拷围 */
    /* max_err锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟筋，锟斤拷锟斤拷锟斤拷锟斤拷锟�0锟斤拷锟斤拷锟斤拷锟斤拷锟狡ｏ拷 */
    if (pid->max_err != 0 && ABS(pid->err[NOW]) > pid->max_err)
        return 0;  // 锟斤拷锟斤拷锟斤拷停止锟斤拷锟�

    /* deadband锟斤拷锟斤拷锟斤拷锟斤拷围锟斤拷锟斤拷锟叫★拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷0锟斤拷锟斤拷止锟今荡ｏ拷 */
    if (pid->deadband != 0 && ABS(pid->err[NOW]) < pid->deadband)
        return 0;  // 锟斤拷锟教★拷锟酵Ｖ癸拷锟斤拷

    /* ========= 位锟斤拷式PID锟斤拷锟斤拷 ========= */
    if (pid->pid_mode == POSITION_PID)
    {
        /* 锟斤拷锟斤拷锟斤拷锟斤拷悖篕p * e(k) */
        pid->pout = pid->Kp * pid->err[NOW];

        /* 锟斤拷锟斤拷锟斤拷锟斤拷悖猴拷奂锟� Ki * e(k) */
        pid->iout += pid->Ki * pid->err[NOW];

        /* 微锟斤拷锟斤拷锟斤拷悖篕d * [e(k) - e(k-1)] */
//        if(pid->flag)
//        {
//            pid->dout= pid->Kd *QEKF_INS.Gyro[1]*57.29578f;
//        }
//
//        else
        {
            pid->dout = pid->Kd * (pid->err[NOW] - pid->err[LAST]);
        }

        /* 锟斤拷锟斤拷锟睫凤拷锟斤拷锟斤拷止锟斤拷锟街憋拷锟斤拷 */
        abs_limit(&(pid->iout), pid->IntegralLimit);

        /* PID锟斤拷锟� = 锟斤拷锟斤拷 + 锟斤拷锟斤拷 + 微锟斤拷 */
        pid->pos_out = pid->pout + pid->iout + pid->dout;

        /* 锟斤拷锟斤拷薹锟斤拷锟斤拷锟街癸拷锟斤拷锟斤拷锟斤拷 */
        abs_limit(&(pid->pos_out), pid->MaxOutput);
        /* 锟斤拷锟芥当前锟斤拷锟斤拷锟轿拷锟揭伙拷蔚锟斤拷锟绞凤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷式PID锟斤拷 */
        pid->last_pos_out = pid->pos_out;


    }
    /* ========= 锟斤拷锟斤拷式PID锟斤拷锟斤拷 ========= */
    else if (pid->pid_mode == DELTA_PID)
    {
        /* 锟斤拷锟斤拷锟筋：Kp * [e(k) - e(k-1)] */
        pid->pout = pid->Kp * (pid->err[NOW] - pid->err[LAST]);

        /* 锟斤拷锟斤拷锟筋：Ki * e(k) */
        pid->iout = pid->Ki * pid->err[NOW];

        /* 微锟斤拷锟筋：Kd * [e(k) - 2e(k-1) + e(k-2)] */
        pid->dout = pid->Kd * (pid->err[NOW] - 2 * pid->err[LAST] + pid->err[LLAST]);

        /* 锟斤拷锟斤拷锟睫凤拷 */
       // abs_limit(&(pid->iout), pid->IntegralLimit);

        /* 锟斤拷锟斤拷锟斤拷锟斤拷 = 锟斤拷锟斤拷 + 锟斤拷锟斤拷 + 微锟斤拷 */
        pid->delta_u = pid->pout + pid->iout + pid->dout;

        /* 锟斤拷前锟斤拷锟� = 锟较达拷锟斤拷锟� + 锟斤拷锟斤拷锟斤拷锟斤拷 */
     if( pid->delta_u>1|| pid->delta_u<-1)
        pid->delta_out = pid->last_delta_out + pid->delta_u;

        /* 锟斤拷锟斤拷薹锟� */
        abs_limit(&(pid->delta_out), pid->MaxOutput);

        /* 锟斤拷锟芥当前锟斤拷锟斤拷锟轿拷锟揭伙拷蔚锟斤拷锟绞凤拷锟斤拷 */
        pid->last_delta_out = pid->delta_out;
    }

    /* ========= 锟斤拷锟斤拷锟斤拷史锟斤拷锟捷ｏ拷锟斤拷锟斤拷锟斤拷锟节ｏ拷 ========= */
    /* 锟斤拷锟斤拷锟绞凤拷锟絃LAST(前前锟斤拷) <- LAST(前锟斤拷) <- NOW(锟斤拷前) */
    pid->err[LLAST] = pid->err[LAST];      // 前前锟斤拷锟斤拷锟� = 前锟斤拷锟斤拷锟�
    pid->err[LAST] = pid->err[NOW];        // 前锟斤拷锟斤拷锟� = 锟斤拷前锟斤拷锟�

    /* 锟斤拷锟斤拷值锟斤拷史 */
    pid->feedback[LLAST] = pid->feedback[LAST];  // 前前锟轿凤拷锟斤拷
    pid->feedback[LAST] = pid->feedback[NOW];    // 前锟轿凤拷锟斤拷

    /* 目锟斤拷值锟斤拷史 */
    pid->target[LLAST] = pid->target[LAST];      // 前前锟斤拷目锟斤拷
    pid->target[LAST] = pid->target[NOW];        // 前锟斤拷目锟斤拷

    /* 锟斤拷锟斤拷PID模式锟斤拷锟斤拷锟斤拷应锟斤拷锟� */
    return pid->pid_mode == POSITION_PID ? pid->pos_out : pid->delta_out;
}

/**
 * @brief 锟斤拷锟絇ID锟斤拷锟斤拷锟斤拷状态
 * @param pid PID锟斤拷锟斤拷锟斤拷锟结构锟斤拷指锟斤拷
 */
void pidClear(PID_TypeDef *pid)
{
    pid->err[NOW] = 0;           // 锟斤拷前锟斤拷锟斤拷锟斤拷锟�
    pid->err[LAST] = 0;          // 前锟斤拷锟斤拷锟斤拷锟斤拷锟�
    pid->err[LLAST] = 0;         // 前前锟斤拷锟斤拷锟斤拷锟斤拷锟�
    pid->last_delta_out = 0;     // 锟较达拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟�
    pid->delta_u = 0;            // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
    pid->last_pos_out = 0;       // 锟较达拷位锟斤拷锟斤拷锟斤拷锟斤拷锟�
    pid->pos_out = 0;            // 锟斤拷前位锟斤拷锟斤拷锟斤拷锟斤拷锟�
    pid->delta_out = 0;          // 锟斤拷前锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟�
    pid->dout = 0;               // 微锟斤拷锟斤拷锟斤拷锟斤拷锟�
    pid->iout = 0;               // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟�
    pid->pout = 0;               // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟�

}

/**
 * @brief 锟斤拷锟斤拷PID锟斤拷锟斤拷锟斤拷锟斤拷始锟斤拷
 * 锟斤拷锟杰ｏ拷锟斤拷始锟斤拷系统锟斤拷锟斤拷锟斤拷锟斤拷要锟斤拷PID锟斤拷锟斤拷锟斤拷
 */
void pidAllInit(void)
{


    // 鎱㈤�熷崟鐜搴ID锛欿p鍔犲ぇ鎻愬崌鍝嶅簲閫熷害
    // Kp=150: 鍊炬枩鏃惰埖鏈鸿浆寰楁洿蹇洿澶�
    // Kd=5: 闃诲凹鍚堥�傦紝鏆備笉璋冩暣
    // 慢速单环角度PID - 恢复原参数
    PID_Init(&Slow_Speed_Balance_AnglePid, POSITION_PID, (float)SLOW_SERVO_DUTY_MAX, GetServoDuty(5), 320.0, 0.0, 40.0);

    /* 直立控制角度环PID - 双环模式外环 */
    // 外环开启，先从小Kp起步
    PID_Init(&Balance_AnglePid, POSITION_PID, 200.0f, 100.0f, -5.0f, 0.0f, 0.0f);
    // 内环初始Kp=30，保守起步
    PID_Init(&Balance_angular_velocityPid, POSITION_PID, (float)SERVO_DUTY_MAX, 600, 8.0f, 0.0f, 0.0f);


    // BackSpeed_Pid 用于后轮速度回路增量PID，增加速度反馈滤波
    PID_Init(&BackSpeed_Pid, DELTA_PID, 2000, 0.0, SPEED_KP, SPEED_KI, 0.0);
}
