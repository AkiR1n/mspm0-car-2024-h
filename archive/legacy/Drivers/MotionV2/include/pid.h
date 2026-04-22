/*
 * pid.h — 通用 PID 控制器（v2 重构版）
 *
 * 相对 v1 (Motor_Encoder_PID/pid.h) 的改动：
 *   1. 把"参数"与"状态"分开 —— pid_params_t vs pid_t。
 *      便于用 const 静态表给多路 PID 统一整定。
 *   2. 显式 dt（秒）参数，不再依赖 10 ms 定时器隐式采样周期。
 *   3. 引入"D on measurement"选项：微分项基于测量值，避免 setpoint 跳变时的微分 kick。
 *   4. D 项带一阶低通（LPF α，0..1，1 = 不滤波），降低编码器/视觉噪声。
 *   5. 反饱和改用 back-calculation：输出饱和时不累加积分（比 v1 的"error × output_i>0"条件更稳健）。
 *   6. 删掉 v1 里声明但从未使用的 prev_error 字段。
 *   7. pid_reset() 只清状态，不清 setpoint（让调用方明确决定）。
 */
#ifndef MOTION_V2_PID_H
#define MOTION_V2_PID_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float kp;
    float ki;
    float kd;

    float out_min;                   /* 输出下限 */
    float out_max;                   /* 输出上限 */
    float int_min;                   /* 积分项下限 */
    float int_max;                   /* 积分项上限 */

    float deriv_lpf_alpha;           /* D 项一阶 LPF 系数 (0..1]，1 = 不滤波 */
    bool  derivative_on_measurement; /* true: D 对测量值求导（推荐），避免 setpoint kick */
} pid_params_t;

typedef struct {
    pid_params_t params;
    float setpoint;
    float integral;
    float last_measurement;
    float last_error;
    float last_deriv_filtered;
    bool  primed;                    /* 首次 step 之前的 guard，避免"虚假"微分 */
} pid_t;

/* 初始化 pid_t：拷贝参数，清零状态。 */
void  pid_init(pid_t *pid, const pid_params_t *params);

/* 仅清状态（integral / last_* / primed）。保留当前 setpoint 和参数。 */
void  pid_reset(pid_t *pid);

/* 运行时修改参数。对 integral / 状态不做重置。 */
void  pid_set_params(pid_t *pid, const pid_params_t *params);

/* 设定目标值。 */
void  pid_set_setpoint(pid_t *pid, float setpoint);

/*
 * 单步计算。
 *   measurement — 当前被控量
 *   dt          — 距离上次 pid_step 的时间（秒），必须 > 0
 *   返回        — 饱和后的控制量，范围 [out_min, out_max]
 */
float pid_step(pid_t *pid, float measurement, float dt);

#endif /* MOTION_V2_PID_H */
