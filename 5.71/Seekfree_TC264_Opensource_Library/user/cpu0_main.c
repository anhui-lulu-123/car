/*********************************************************************************************************************
* 拨码开关 P33_11：
*   拨上（LOW）= 慢速模式：固定开环指令800，单环PID
*   拨下（HIGH）= 快速模式：遥控器CH1转向/CH2调速，双环PID
*
*   KEY1(P20_6)  发车 / 停车（对齐1and2）
*   KEY2(P20_7)  惯导/状态重置
*   KEY3(P11_2)  记录航点
*   KEY4(P11_3)  全部重置
*         - 未使能时（wp_count>0）：位置归零、保留航点、使能导航
*         - 已使能时：全部重置，回到录制模式
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "QuaternionEKF.h"
#include "Control.h"
#include "PID.h"
#include "Servo.h"
#include "INS_Nav.h"
#include "Remote_Control.h"
#include "Attitude.h"

#define BUZZER_PIN      (P33_10)
#define MODE_SWITCH_PIN (P33_11)
#define SLOW_MODE_SPEED      600     // 慢速模式默认速度
#define SLOW_SPEED_MIN       200     // 慢速模式速度下限（已降低）
#define SLOW_SPEED_MAX       1000    // 慢速模式速度上限
#define SLOW_SPEED_STEP      50      // 每次按键调速步进

#define BALANCE_ZERO_BASE  -0.7f    // 本次先只用一个中值，便于先跑起来
#define SLOW_BALANCE_ZERO  BALANCE_ZERO_BASE
#define FAST_BALANCE_ZERO  BALANCE_ZERO_BASE

// 科目二超低速模式：拨码上+KEY2按住
#define EXAM2_MODE_SPEED    200     // 科目二目标速度（极低）
#define EXAM2_MODE_KEY2_PRESSED  (gpio_get_level(P20_7) == 0)

#pragma section all "cpu0_dsram"

int core0_main(void)
{
    static int16 slow_mode_speed_set = SLOW_MODE_SPEED;
    static uint8 exam2_mode_active = 0;  // 科目二模式标志

    clock_init();
    debug_init();
    SUM_Init();
    gpio_init(BUZZER_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    cpu_wait_event_ready();

    // 添加调试输出：确认初始化完成
    small_driver_send_string("Init complete, entering main loop\n");

    while (TRUE)
    {
        /* ===== 拨码开关+KEY2：切换模式 ===== */
        uint8 mode_switch_low = (gpio_get_level(MODE_SWITCH_PIN) == GPIO_LOW);

        // 切换逻辑：KEY2按下时锁定科目二模式，KEY2松开不解除（直到下次按KEY2）
        static uint8 exam2_mode_locked = 0;
        static uint8 key2_last = 1;
        uint8 key2_now = gpio_get_level(P20_7);

        // 检测KEY2的下降沿（刚按下）
        if (key2_now == 0 && key2_last == 1 && mode_switch_low)
        {
            exam2_mode_locked = 1;  // 按下KEY2+拨码上→锁定科目二模式
        }
        key2_last = key2_now;

        // 科目二模式：拨码上且被KEY2锁定
        if (mode_switch_low && exam2_mode_locked)
        {
            exam2_mode_active = 1;
            car_mode_switch_num = 0;
            target_speed = EXAM2_MODE_SPEED;
            Balance_init_zero = SLOW_BALANCE_ZERO;
        }
        else if (mode_switch_low)
        {
            exam2_mode_active = 0;
            car_mode_switch_num = 0;
            target_speed = slow_mode_speed_set;
            Balance_init_zero = SLOW_BALANCE_ZERO;
        }
        else
        {
            exam2_mode_active = 0;
            exam2_mode_locked = 0;  // 拨码下时清除锁定
            car_mode_switch_num = 1;
            Balance_init_zero = FAST_BALANCE_ZERO;
        }

        /* ===== 同步科目二模式到导航模块 ===== */
        INS_Nav_SetExam2Mode(exam2_mode_active);

        /* ===== KEY1: 发车 / 停车（对齐 1and2：KEY1=P20_6） ===== */
        static uint8 key1_last = 1;
        uint8 key1_now = gpio_get_level(P20_6);
        if (key1_now == 0 && key1_last == 1)
        {
            if (Bike_State == Run)
            {
                Bike_State = Stop;
            }
            else
            {
                if (!nav_enabled && wp_count > 0)
                    INS_Nav_Commit();   // 位置归零，使能导航
                Bike_State = Run;
            }
        }
        key1_last = key1_now;

        /* ===== KEY3: 录制航点 / 启动自动录点 ===== */
        static uint8 key3_last = 1;
        static uint16 key3_press_time = 0;
        uint8 key3_now = gpio_get_level(P11_2);
        if (key3_now == 0 && key3_last == 1)
        {
            key3_press_time = 0;  // 开始计时
        }
        else if (key3_now == 0 && key3_last == 0)
        {
            key3_press_time++;  // 累加计时
        }
        else if (key3_now == 1 && key3_last == 0 && key3_press_time > 0)
        {
            // KEY3释放
            if (key3_press_time > 20)  // 长按超过约1秒
            {
                // 长按：切换自动录点模式
                if (car_mode_switch_num == 0)
                {
                    if (auto_wp_enabled)
                        INS_Nav_StopAutoWaypoint();
                    else
                        INS_Nav_StartAutoWaypoint();
                }
            }
            else if (key3_press_time > 0)  // 短按
            {
                // 短按：手动录点 + 调速（非科目二模式）
                if (car_mode_switch_num == 0)
                {
                    INS_Nav_RecordWaypoint();
                    if (!exam2_mode_active)
                    {
                        slow_mode_speed_set += SLOW_SPEED_STEP;
                        slow_mode_speed_set = LIMIT(slow_mode_speed_set, SLOW_SPEED_MIN, SLOW_SPEED_MAX);
                        target_speed = slow_mode_speed_set;
                    }
                }
            }
            key3_press_time = 0;
        }
        key3_last = key3_now;

        /* ===== KEY4: 慢速模式减速 / 快速模式复位导航（长按1秒解锁科目二） ===== */
        static uint8 key4_last = 1;
        static uint16 key4_press_time = 0;
        uint8 key4_now = gpio_get_level(P11_3);
        if (key4_now == 0 && key4_last == 1)
        {
            key4_press_time = 0;  // 开始计时
        }
        else if (key4_now == 0 && key4_last == 0)
        {
            key4_press_time++;  // 累加计时（约50ms一次）
        }
        else if (key4_now == 1 && key4_last == 0 && key4_press_time > 0)
        {
            // KEY4释放
            if (key4_press_time > 20)  // 长按超过约1秒
            {
                exam2_mode_locked = 0;  // 解锁科目二模式
            }
            else if (key4_press_time > 0)  // 短按
            {
                if (car_mode_switch_num == 0 && !exam2_mode_active)
                {
                    slow_mode_speed_set -= SLOW_SPEED_STEP;
                    slow_mode_speed_set = LIMIT(slow_mode_speed_set, SLOW_SPEED_MIN, SLOW_SPEED_MAX);
                    target_speed = slow_mode_speed_set;
                }
                else if (car_mode_switch_num == 1)
                {
                    INS_Nav_Reset();  // 快速模式复位导航
                }
            }
            key4_press_time = 0;
        }
        key4_last = key4_now;

        /* ===== 遥控器 + 导航：决定方向和速度 ===== */
        Remote_Control();
        INS_Nav_Navigate();   // 50ms 更新 nav_direction_bias

        /* ===== 蜂鸣器：到达航点鸣一次 ===== */
        static uint8 buzzer_ticks = 0;
        if (wp_arrived_flag)
        {
            wp_arrived_flag = 0;
            gpio_set_level(BUZZER_PIN, GPIO_HIGH);
            buzzer_ticks = 3;   // 3×50ms = 150ms
        }
        if (buzzer_ticks > 0 && --buzzer_ticks == 0)
            gpio_set_level(BUZZER_PIN, GPIO_LOW);

        if(car_mode_switch_num == 1)  // 快速模式
        {
            if(nav_enabled)
            {
                Direction_Pid.pos_out = nav_direction_bias;
                if(Remote_Connect_State)
                    target_speed = Remote_Speed_Target;  // 速度仍由遥控器控制
            }
            else
            {
                Direction_Pid.pos_out = Remote_Connect_State ? Remote_Dicretion_Targrt : 0.0f;
                if(Remote_Connect_State)
                    target_speed = Remote_Speed_Target;
            }
        }
        else  // 慢速模式
        {
            Direction_Pid.pos_out = nav_enabled ? nav_direction_bias : 0.0f;
        }

        /* 电机指令已在 7ms 中断中发送，20ms 中断负责速度请求 */
        
        /* ===== TFT 显示 ===== */
        tft180_show_string(0,  0, exam2_mode_active ? "MODE:EXAM2" : (car_mode_switch_num == 0 ? "MODE:SLOW" : "MODE:FAST"));
        tft180_show_string(88, 0, "MDBG4");

        // 添加调试输出：确认显示更新
        static uint32 loop_count = 0;
        if (loop_count % 100 == 0) {  // 每100次循环输出一次
            small_driver_send_string("Display updated, loop: ");
            small_driver_send_int(loop_count);
            small_driver_send_string("\n");
        }
        loop_count++;

        tft180_show_string(0,  20, "Pit:");
        tft180_show_float (32, 20, QEKF_INS.Pitch, 2, 2);

        tft180_show_string(0,  40, "Spd:");
        tft180_show_int   (32, 40, target_speed, 4);

        tft180_show_string(0,  60, "Srv:");
        tft180_show_float (32, 60, GetServoAngle(servo_input_duty), 2, 1);

        tft180_show_string(0,  80, Remote_Connect_State ? "RC:ON " : "RC:OFF");
        tft180_show_string(64, 80, Bike_State == Run ? "MTR:ON " : "MTR:OFF");

        tft180_show_string(0, 100, Bike_State == Run ? "STATE:RUN" : "STATE:STP");

        // 导航状态：NAV:ON/OFF + 当前/总航点 + 自动录点
        tft180_show_string(0,  120, nav_enabled ? "NAV:ON " : "NAV:OFF");
        tft180_show_string(64, 120, auto_wp_enabled ? "AUTO " : "    ");
        tft180_show_int   (96, 120, wp_current + (nav_enabled ? 1 : 0), 1);
        tft180_show_string(104, 120, "/");
        tft180_show_int   (112,120, wp_count, 1);

        // 惯导位置：X / Y（单位：米）
        tft180_show_string(0,  140, "X:");
        tft180_show_float (16, 140, ins_pos_x, 2, 1);
        tft180_show_string(72, 140, "Y:");
        tft180_show_float (88, 140, ins_pos_y, 2, 1);

        // VOFA+ FireWater协议: Pitch, Srv, GyroY, PosX, PosY, NavBias
        printf("%.2f,%.2f,%.3f,%.2f,%.2f,%.3f\n",
               QEKF_INS.Pitch,
               GetServoAngle(servo_input_duty),
               QEKF_INS.Gyro[1] * 57.29578f,
               ins_pos_x,
               ins_pos_y,
               nav_direction_bias);

        system_delay_ms(50);
    }
}

#pragma section all restore
