/*
 * motor.h — 双 H 桥直流电机驱动（v2 重构版）
 *
 * 相对 v1 (Motor_Encoder_PID/Motor.h/.c) 的改动：
 *   1. 用 motor_id_t 枚举替代 MOTOR_A/B/ALL 宏（宏在 .h 和 .c 里重复定义过）。
 *      A/B 语义不明 → 改叫 MOTOR_LEFT/RIGHT，左右语义与底盘机械一致。
 *   2. 统一速度标度为 [-1.0, +1.0]：
 *      - 负值 = 反转，正值 = 正转，0 = 停止
 *      - 删掉 v1 里那个令人困惑的 Motor_Set_Pwm(float, float) 吃 -100..100 的旧 API
 *   3. 死区 / 平衡因子 / PWM 通道分配 改为运行时 motor_config_t，不再硬编码。
 *   4. 删除从未调用的 Motor_PWM_Stop（死代码）。
 *   5. 新增 MOTOR_DIR_BRAKE（短路刹车），供未来高级机动使用。
 *   6. 所有函数有明确的幂等 / 参数校验行为，无效 id 直接 return（不再悄悄跑别的电机）。
 */
#ifndef MOTION_V2_MOTOR_H
#define MOTION_V2_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    MOTOR_LEFT  = 0,
    MOTOR_RIGHT = 1,
    MOTOR_COUNT = 2,
} motor_id_t;

typedef enum {
    MOTOR_DIR_COAST   =  0,   /* 两端悬空，靠惯性滑行 */
    MOTOR_DIR_FORWARD = +1,
    MOTOR_DIR_REVERSE = -1,
    MOTOR_DIR_BRAKE   =  2,   /* 两端都拉低，短路刹车 */
} motor_dir_t;

typedef struct {
    /*
     * 最小有效占空比：若 |speed| ∈ (0, min_duty)，强制抬升到 min_duty，
     * 避免在极低速度下电机卡死或抖动。设 0 则禁用。
     * v1 硬编码 0.1。
     */
    float min_duty;

    /*
     * 右电机相对左电机的速度标定因子。用于补偿机械左右不对称。
     * 1.0 = 无补偿。v1 硬编码 1.0。
     */
    float right_balance;
} motor_config_t;

extern const motor_config_t MOTOR_CONFIG_DEFAULT;

/*
 * 初始化电机：停止所有电机 + 启动 PWM 定时器。
 * 幂等：多次调用只启动定时器一次。
 */
void motor_init(const motor_config_t *cfg);

/*
 * 直接指定电机方向 + 占空比。供高级用法使用。
 *   id        — 电机 ID
 *   dir       — 方向枚举
 *   duty      — 占空比 [0, 1]（仅在 dir=FORWARD/REVERSE 时使用）
 */
void motor_set_raw(motor_id_t id, motor_dir_t dir, float duty);

/*
 * 按带符号速度驱动电机。
 *   speed ∈ [-1, +1]，负值反转，正值正转，0 停止。
 * 内部处理死区、饱和、平衡因子。
 */
void motor_set(motor_id_t id, float speed);

/* 同时设定左右轮速度。等价于两次 motor_set。 */
void motor_set_pair(float left, float right);

/* 停止单路（coast）。 */
void motor_stop(motor_id_t id);

/* 停止所有电机。 */
void motor_stop_all(void);

/* 短路刹车所有电机（主动减速，不能长时间保持以免发热）。 */
void motor_brake_all(void);

#endif /* MOTION_V2_MOTOR_H */
