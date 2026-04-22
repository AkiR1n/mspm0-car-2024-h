/*
 * motion_control.h — 底盘高级运动控制（v2 重构版）
 *
 * 相对 v1 (Motor_Encoder_PID/motor_control.h/.c) 的改动：
 *   1. 命名从 MotorControl_* → motion_*（更贴"运动控制"语义，和 motor.c 这类底层区分开）。
 *   2. PID 参数从 #define → motion_config_t 运行时结构体，便于统一整定。
 *   3. 速度单位从"百分比 0..100" → "PPS（脉冲/秒）"物理量，闭环目标直接对应编码器反馈。
 *   4. 解除对 mpu6050 全局 `yaw` 的隐式依赖：
 *      改由 caller（sensor_task）调 motion_set_yaw_feedback(yaw_deg) 注入。
 *   5. monolithic `MotorControl_Update` switch 拆成每个 mode 一个私有静态函数。
 *   6. 显式 dt 参数 → 上层能准确知道并控制周期。
 *   7. motion_init 不再调 LineTracker_Init / Motor_Init / Encoder_Init，
 *      改为由 caller 先显式初始化依赖模块。让组合关系清晰。
 *   8. 删除 v1 里被注释掉的"边缘传感器硬分类"代码（一年未启用的死逻辑）。
 */
#ifndef MOTION_V2_MOTION_CONTROL_H
#define MOTION_V2_MOTION_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include "pid.h"

typedef enum {
    MOTION_MODE_STOP,
    MOTION_MODE_LINE_FOLLOW,    /* 循迹 PID —— 位置偏差 → 差速修正 */
    MOTION_MODE_YAW_HOLD,       /* Yaw 角保持 —— 陀螺角偏差 → 差速修正 */
    MOTION_MODE_DIFFERENTIAL,   /* 直接差速 —— 左右轮独立目标 PPS */
    MOTION_MODE_OPEN_LOOP,      /* 开环 —— 左右轮均按 base_speed 跑，无闭环 */
} motion_mode_t;

typedef struct {
    pid_params_t line_pid;          /* 循迹位置 → 差速 */
    pid_params_t yaw_pid;           /* yaw → 差速 */
    pid_params_t wheel_speed_pid_l; /* 左轮速度闭环：PPS 目标 → 归一化电机 speed */
    pid_params_t wheel_speed_pid_r; /* 右轮速度闭环 */
    float        line_correction_scale; /* 循迹 PID 输出 × 此值 = 两轮 PPS 差值 */
    float        max_wheel_pps;         /* 单轮最大 PPS，超过后饱和 */
} motion_config_t;

extern const motion_config_t MOTION_CONFIG_DEFAULT;

/*
 * 初始化。本函数**不调**底层驱动的 init —— caller 先自己初始化：
 *   motor_init(NULL);
 *   encoder_init(NULL);
 *   linetracker_init(NULL);
 *   motion_init(NULL);
 */
void motion_init(const motion_config_t *cfg);

void motion_set_mode(motion_mode_t mode);

/* 基础速度（PPS）。循迹/Yaw/开环 mode 下作为左右轮共同的"前进速度"。 */
void motion_set_base_speed_pps(float pps);

/* 设定 yaw hold 模式的目标角（度）。 */
void motion_set_target_yaw(float yaw_deg);

/* 注入 yaw 反馈（度）。解耦 mpu6050 依赖：传感器任务周期调用。 */
void motion_set_yaw_feedback(float yaw_deg);

/* 差速模式下手动设置左右轮目标 PPS。 */
void motion_set_differential(float left_pps, float right_pps);

/*
 * 周期推进一次闭环。dt 单位 秒。典型周期 10 ms（即 dt=0.01）。
 * caller 需要在本函数调用前：
 *   - 刷新 linetracker_read() （若 mode=LINE_FOLLOW）
 *   - 调 motion_set_yaw_feedback() （若 mode=YAW_HOLD）
 */
void motion_step(float dt);

/* 紧急停止。清空所有 PID 状态。 */
void motion_stop(void);

#endif /* MOTION_V2_MOTION_CONTROL_H */
