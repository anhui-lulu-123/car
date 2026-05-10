/*
 * INS_Nav.h
 *
 * 简易惯导导航模块：
 *   - 用小刷板回传的轮速（RPM）估计车体前进速度；
 *   - 用四元数 EKF 输出的航向角 Yaw；
 *   - 在 5ms 周期内对平面位置 (x, y) 做数值积分；
 *   - 在主循环中根据当前位置与航点做方位跟踪，输出 nav_direction_bias 给转向环。
 */

#ifndef CODE_INS_NAV_H_
#define CODE_INS_NAV_H_

#include "zf_common_headfile.h"

/* ====================== 物理参数（需按实车标定）====================== */
#define WHEEL_RADIUS_M      0.038f    /* 驱动轮有效半径(m)，里程不对时微调 */
#define INS_DT              0.005f    /* 位置积分步长(s)，与 CCU60_CH1 周期一致：5ms */

/* RPM → 轮缘线速度(m/s)：v = 2πr·RPM/60 */
#define RPM_TO_MS           (2.0f * 3.14159265f * WHEEL_RADIUS_M / 60.0f)

/* ====================== 导航参数 ====================== */
#define MAX_WAYPOINTS       8         /* 最多保存航点数 */
#define WP_ARRIVE_DIST_M    0.50f     /* 判定抵达当前航点的平面距离阈值(m) */
#define NAV_KP              0.015f    /* 航向误差(deg) → 倾斜偏置(deg)的比例增益 */
#define NAV_MAX_BIAS        1.5f      /* nav_direction_bias 限幅(deg)，防止过猛转向 */

/* ====================== 科目二模式参数（超低速精确） ====================== */
#define EXAM2_NAV_KP        0.008f    /* 科目二：更小的增益，更柔和 */
#define EXAM2_NAV_MAX_BIAS  0.8f      /* 科目二：更小的最大偏置 */
#define EXAM2_WP_ARRIVE_DIST_M 0.30f  /* 科目二：更小的到达判定距离 */

/* ====================== 自动录点参数 ====================== */
#define AUTO_WP_DIST_M      0.30f     /* 自动录点：移动此距离自动记录一个新点 */

/* ====================== 数据类型 ====================== */
typedef struct {
    float x;   /* 东向坐标(m)，与积分定义一致 */
    float y;   /* 北向坐标(m) */
    float yaw; /* 到达此点时的目标航向角(deg)，用于导航控制 */
} WaypointXY_t;

/* ====================== 状态变量（模块内写、外部多读） ====================== */
extern float ins_pos_x;           /* 当前估计位置 X(m) */
extern float ins_pos_y;           /* 当前估计位置 Y(m) */
extern float ins_speed_ms;        /* 当前估计车速(m/s)，由 RPM 换算 */

extern WaypointXY_t waypoints[MAX_WAYPOINTS];
extern int wp_count;              /* 已录制航点个数 */
extern int wp_current;            /* 当前追踪的航点索引 [0, wp_count) */
extern float nav_direction_bias;  /* 输出给 Direction_Pid.pos_out 的倾角偏置(deg) */

extern uint8 nav_enabled;         /* 1：参与导航；0：不外挂航向修正 */
extern uint8 wp_arrived_flag;     /* 每到一站置 1，cpu0_main 读后应清零 */
extern uint8 auto_wp_enabled;      /* 自动录点使能标志 */

/* ====================== 接口 ====================== */

/* 初始化；在 SUM_Init() 末尾调用 */
void INS_Nav_Init(void);

/* 5ms 调用一次（isr.c → CCU60_CH1）：速度 × 航向 → 位置积分 */
void INS_Nav_Update(void);

/* 约 50ms 调用一次（cpu0_main 主循环）：算方位误差 → nav_direction_bias */
void INS_Nav_Navigate(void);

/* 把当前 (ins_pos_x, ins_pos_y) 记入航点序列（KEY3 短按） */
void INS_Nav_RecordWaypoint(void);

/* 清空全部航点并复位位置/导航状态（KEY4：整体重置场景） */
void INS_Nav_Reset(void);

/* 提交导航：坐标归零、保留航点、wp_current=0、nav_enabled=1（首次发车前 INS_Nav_Commit） */
void INS_Nav_Commit(void);

void INS_Nav_Enable(uint8 en);

/* 启用自动录点：移动AUTO_WP_DIST_M距离自动记录一个新点 */
void INS_Nav_StartAutoWaypoint(void);

/* 停止自动录点 */
void INS_Nav_StopAutoWaypoint(void);

/* 获取剩余航点容量 */
int INS_Nav_GetRemainingSlots(void);
void INS_Nav_SetExam2Mode(uint8 exam2_en);

/* 科目二超低速精确模式控制 */

/* 车体当前位置到「当前目标航点」的直线距离(m) */
float INS_Nav_GetDistToTarget(void);

/* 角度差 from_deg → to_deg，归一化到 (-180°, 180°] */
float INS_Nav_HeadingDiff(float from_deg, float to_deg);

#endif /* CODE_INS_NAV_H_ */
