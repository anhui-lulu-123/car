/*
 * INS_Nav.c —— 纯惯导平面导航
 *
 * 与其它模块的配合：
 *   Attitude / QEKF_INS      ：提供 Yaw（deg），定义车头朝向；
 *   small_driver             ：小刷板 UART 回传 motor_value.receive_*_speed_data；
 *   Control.Direction_Pid    ：导航开启时 pos_out 由 nav_direction_bias 覆盖；
 *   cpu0_main                ：50ms 周期 Remote_Control / INS_Nav_Navigate / 显示。
 *
 * 坐标约定（平面近似）：
 *   X 轴：车头指向「正东」时前进方向在 XY 平面上的投影；
 *   Y 轴：与之正交（典型为「北」向），与 atan2(dy,dx) 的工程惯例一致。
 *   积分：Δx = v·cos(yaw)·Δt ，Δy = v·sin(yaw)·Δt ，Yaw 与 EKF 定义一致。
 */

#include "INS_Nav.h"
#include "QuaternionEKF.h"
#include "Control.h"
#include "small_driver_uart_control.h"

/* ====================== 状态变量 ====================== */
float ins_pos_x = 0.0f;
float ins_pos_y = 0.0f;
float ins_speed_ms = 0.0f;

WaypointXY_t waypoints[MAX_WAYPOINTS];
int wp_count   = 0;
int wp_current = 0;

float nav_direction_bias = 0.0f;
uint8 nav_enabled = 0;
uint8 wp_arrived_flag = 0;
uint8 exam2_mode = 0;  // 科目二超低速模式标志

/* ====================== 自动录点状态 ====================== */
uint8 auto_wp_enabled = 0;              // 自动录点使能标志（已在h文件extern）
static float last_wp_x = 0.0f;             // 上一个记录点的X坐标
static float last_wp_y = 0.0f;             // 上一个记录点的Y坐标

/* ====================== 内部工具函数 ====================== */

float INS_Nav_HeadingDiff(float from_deg, float to_deg)
{
    float diff = from_deg - to_deg;
    while (diff >  180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}

float INS_Nav_GetDistToTarget(void)
{
    if (wp_count == 0) return 0.0f;
    float dx = waypoints[wp_current].x - ins_pos_x;
    float dy = waypoints[wp_current].y - ins_pos_y;
    return sqrtf(dx * dx + dy * dy);
}

/* ====================== 对外接口 ====================== */

void INS_Nav_Init(void)
{
    ins_pos_x = 0.0f;
    ins_pos_y = 0.0f;
    ins_speed_ms = 0.0f;
    wp_count    = 0;
    wp_current  = 0;
    nav_direction_bias = 0.0f;
    nav_enabled = 0;
}

void INS_Nav_Enable(uint8 en)
{
    nav_enabled = en;
    if (!en)
    {
        nav_direction_bias = 0.0f;
    }
}

void INS_Nav_SetExam2Mode(uint8 exam2_en)
{
    exam2_mode = exam2_en;
}

void INS_Nav_Reset(void)
{
    ins_pos_x = 0.0f;
    ins_pos_y = 0.0f;
    wp_count   = 0;
    wp_current = 0;
    nav_direction_bias = 0.0f;
    nav_enabled = 0;
}

/*
 * 录制结束后第一次正式跑线前调用：
 *   只把积分坐标拉回起点，航点列表保留；
 *   wp_current 指回第 0 个点，并打开 nav_enabled。
 */
void INS_Nav_Commit(void)
{
    ins_pos_x  = 0.0f;
    ins_pos_y  = 0.0f;
    wp_current = 0;
    nav_enabled = 1;
}

void INS_Nav_RecordWaypoint(void)
{
    if (wp_count >= MAX_WAYPOINTS) return;
    waypoints[wp_count].x = ins_pos_x;
    waypoints[wp_count].y = ins_pos_y;
    waypoints[wp_count].yaw = QEKF_INS.Yaw;  // 记录到达此点时的航向角
    wp_count++;
    /* 录第一个点时立刻令追踪索引指向它 */
    if (wp_count == 1) wp_current = 0;
}

/* 启用自动录点：移动AUTO_WP_DIST_M距离自动记录一个新点 */
void INS_Nav_StartAutoWaypoint(void)
{
    if (wp_count >= MAX_WAYPOINTS) return;
    INS_Nav_RecordWaypoint();  // 先记录第一个点作为参考
    last_wp_x = waypoints[wp_count-1].x;
    last_wp_y = waypoints[wp_count-1].y;
    auto_wp_enabled = 1;
}

/* 停止自动录点 */
void INS_Nav_StopAutoWaypoint(void)
{
    auto_wp_enabled = 0;
}

/* 获取剩余航点容量 */
int INS_Nav_GetRemainingSlots(void)
{
    return MAX_WAYPOINTS - wp_count;
}

/*
 * INS_Nav_Update —— 5ms（CCU60_CH1 ISR）
 *
 * 车速来源：小刷板左右通道反馈（RPM）。
 *   单电机可能接在左或右通道：两侧都有值时取平均，只有一侧有值时取该侧。
 *
 * 简化假设：用车速标量 × Yaw 方向做平面积分，未建模侧滑；低速平衡车通常可接受。
 */
void INS_Nav_Update(void)
{
    int16 left_speed = motor_value.receive_left_speed_data;
    int16 right_speed = motor_value.receive_right_speed_data;
    int16 speed_rpm = 0;

    if (abs(left_speed) > 1 && abs(right_speed) > 1)
        speed_rpm = (int16)((left_speed + right_speed) / 2);
    else if (abs(left_speed) > 1)
        speed_rpm = left_speed;
    else
        speed_rpm = right_speed;

    ins_speed_ms = (float)speed_rpm * RPM_TO_MS;

    float yaw_rad = QEKF_INS.Yaw * 0.01745329f;  /* deg → rad */

    ins_pos_x += ins_speed_ms * cosf(yaw_rad) * INS_DT;
    ins_pos_y += ins_speed_ms * sinf(yaw_rad) * INS_DT;

    /* 自动录点：移动一定距离自动记录一个新点 */
    if (auto_wp_enabled && wp_count < MAX_WAYPOINTS)
    {
        float dx = ins_pos_x - last_wp_x;
        float dy = ins_pos_y - last_wp_y;
        float dist = sqrtf(dx * dx + dy * dy);
        if (dist >= AUTO_WP_DIST_M)
        {
            INS_Nav_RecordWaypoint();
            last_wp_x = waypoints[wp_count-1].x;
            last_wp_y = waypoints[wp_count-1].y;
        }
    }
}

/*
 * INS_Nav_Navigate —— 建议 50ms（cpu0_main）
 *
 * 前置条件：nav_enabled && 已有航点 && Bike_State==Run。
 *   否则清零 nav_direction_bias，交由遥控器或其它逻辑控制方向。
 *
 * 算法概要：
 *   1. 若到当前 wp_current 距离 < WP_ARRIVE_DIST_M：判到达 → 蜂鸣/索引递增；
 *   2. 最后一个航点到达后停车（Bike_State=Stop），bias 清零；
 *   3. 否则算目标方位角 bearing = atan2(dy, dx)，与当前 Yaw 比较差；
 *   4. heading_err * NAV_KP → nav_direction_bias，并限幅 NAV_MAX_BIAS。
 *
 * arrive_cooldown：防止在阈值附近抖动导致重复触发「到站」。
 */
void INS_Nav_Navigate(void)
{
    if (!nav_enabled || wp_count == 0 || Bike_State != Run)
    {
        nav_direction_bias = 0.0f;
        return;
    }

    static uint8 arrive_cooldown = 0;
    if (arrive_cooldown > 0)
    {
        arrive_cooldown--;
    }
    else
    {
        float arrive_dist = exam2_mode ? EXAM2_WP_ARRIVE_DIST_M : WP_ARRIVE_DIST_M;
        if (INS_Nav_GetDistToTarget() < arrive_dist)
        {
            wp_arrived_flag = 1;
            arrive_cooldown = 8;    /* 8 × 50ms ≈ 0.4s */
            if (wp_current < wp_count - 1)
            {
                wp_current++;
            }
            else
            {
                nav_direction_bias = 0.0f;
                Bike_State = Stop;
                return;
            }
        }
    }

    /* 目标相对矢量 → 方位角（deg）：x 轴为 0°，逆时针为正（数学角） */
    float dx = waypoints[wp_current].x - ins_pos_x;
    float dy = waypoints[wp_current].y - ins_pos_y;
    float target_yaw = waypoints[wp_current].yaw;  // 使用记录的目标航向角

    /* 当前车头朝向与目标航向的误差 */
    float heading_err = INS_Nav_HeadingDiff(QEKF_INS.Yaw, target_yaw);

    /* 根据模式选择不同的增益和限幅 */
    float nav_kp = exam2_mode ? EXAM2_NAV_KP : NAV_KP;
    float nav_max_bias = exam2_mode ? EXAM2_NAV_MAX_BIAS : NAV_MAX_BIAS;

    nav_direction_bias = heading_err * nav_kp;
    if (nav_direction_bias >  nav_max_bias) nav_direction_bias =  nav_max_bias;
    if (nav_direction_bias < -nav_max_bias) nav_direction_bias = -nav_max_bias;
}
