#include "motion_control.h"
#include "motor.h"
#include "encoder.h"
#include "linetracker.h"
#include <stddef.h>

/* ─── 默认参数（迁自 v1 motor_control.c 的 #define） ─── */
const motion_config_t MOTION_CONFIG_DEFAULT = {
    .line_pid = {
        .kp = 1.2f, .ki = 0.0f, .kd = 0.6f,
        .out_min = -20.0f, .out_max = 20.0f,
        .int_min = -100.0f, .int_max = 100.0f,
        .deriv_lpf_alpha = 1.0f,
        .derivative_on_measurement = false,    /* 循迹 setpoint 恒为 0，D on error 也无 kick */
    },
    .yaw_pid = {
        .kp = 1.0f, .ki = 0.0f, .kd = 0.2f,
        .out_min = -100.0f, .out_max = 100.0f,
        .int_min = -200.0f, .int_max = 200.0f,
        .deriv_lpf_alpha = 0.7f,
        .derivative_on_measurement = true,
    },
    .wheel_speed_pid_l = {
        .kp = 0.0012f, .ki = 0.002f, .kd = 0.0001f,   /* PPS 级别，v1 是 0..100 百分比，参数值相应缩放 */
        .out_min = -1.0f, .out_max = 1.0f,            /* 输出直接喂 motor_set */
        .int_min = -500.0f, .int_max = 500.0f,
        .deriv_lpf_alpha = 0.5f,
        .derivative_on_measurement = true,
    },
    .wheel_speed_pid_r = {
        .kp = 0.0012f, .ki = 0.002f, .kd = 0.0001f,
        .out_min = -1.0f, .out_max = 1.0f,
        .int_min = -500.0f, .int_max = 500.0f,
        .deriv_lpf_alpha = 0.5f,
        .derivative_on_measurement = true,
    },
    .line_correction_scale = 20.0f,     /* 循迹 PID 输出 1 → ±20 PPS 差速修正 */
    .max_wheel_pps         = 3000.0f,   /* 轮子最大 3000 PPS（视硬件标定） */
};

/* ─── 内部状态 ─── */
static motion_config_t s_cfg;
static motion_mode_t   s_mode = MOTION_MODE_STOP;

static pid_t s_line_pid;
static pid_t s_yaw_pid;
static pid_t s_spd_pid_l;
static pid_t s_spd_pid_r;

static float s_base_speed_pps      = 0.0f;
static float s_target_yaw_deg      = 0.0f;
static float s_yaw_feedback_deg    = 0.0f;
static float s_diff_left_target    = 0.0f;
static float s_diff_right_target   = 0.0f;

static inline float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* ─── 初始化 ─── */
void motion_init(const motion_config_t *cfg)
{
    s_cfg = (cfg != NULL) ? *cfg : MOTION_CONFIG_DEFAULT;

    pid_init(&s_line_pid,  &s_cfg.line_pid);
    pid_init(&s_yaw_pid,   &s_cfg.yaw_pid);
    pid_init(&s_spd_pid_l, &s_cfg.wheel_speed_pid_l);
    pid_init(&s_spd_pid_r, &s_cfg.wheel_speed_pid_r);

    /* 循迹 PID 的目标永远是 0（偏差为 0 = 线在正中）*/
    pid_set_setpoint(&s_line_pid, 0.0f);
    pid_set_setpoint(&s_yaw_pid,  0.0f);

    s_base_speed_pps    = 0.0f;
    s_target_yaw_deg    = 0.0f;
    s_yaw_feedback_deg  = 0.0f;
    s_diff_left_target  = 0.0f;
    s_diff_right_target = 0.0f;
    s_mode              = MOTION_MODE_STOP;
}

void motion_set_mode(motion_mode_t mode)
{
    if (mode == s_mode) return;
    s_mode = mode;
    /* 切换到 STOP 立即停；其它模式让下一次 motion_step 生效。*/
    if (mode == MOTION_MODE_STOP) motion_stop();
}

void motion_set_base_speed_pps(float pps)
{
    s_base_speed_pps = clampf(pps, 0.0f, s_cfg.max_wheel_pps);
}

void motion_set_target_yaw(float yaw_deg)
{
    s_target_yaw_deg = yaw_deg;
    pid_set_setpoint(&s_yaw_pid, yaw_deg);
}

void motion_set_yaw_feedback(float yaw_deg)
{
    s_yaw_feedback_deg = yaw_deg;
}

void motion_set_differential(float left_pps, float right_pps)
{
    s_diff_left_target  = clampf(left_pps,  -s_cfg.max_wheel_pps, s_cfg.max_wheel_pps);
    s_diff_right_target = clampf(right_pps, -s_cfg.max_wheel_pps, s_cfg.max_wheel_pps);
}

/* ─── 每种 mode 的目标 PPS 生成器 ─── */

static void mode_line_follow_targets(float dt, float *out_l, float *out_r)
{
    const int16_t pos = linetracker_position();   /* 假设 caller 已 linetracker_read */
    const float   corr = pid_step(&s_line_pid, (float)pos, dt);
    /* corr 正 → 线在右 → 需要右转 → 右轮减速，左轮加速 */
    const float diff = corr * s_cfg.line_correction_scale;
    *out_l = s_base_speed_pps + diff;
    *out_r = s_base_speed_pps - diff;
}

static void mode_yaw_hold_targets(float dt, float *out_l, float *out_r)
{
    const float corr = pid_step(&s_yaw_pid, s_yaw_feedback_deg, dt);
    *out_l = s_base_speed_pps - corr;
    *out_r = s_base_speed_pps + corr;
}

static void mode_differential_targets(float *out_l, float *out_r)
{
    *out_l = s_diff_left_target;
    *out_r = s_diff_right_target;
}

static void mode_open_loop(void)
{
    /* 不走速度闭环，直接按 base_speed 的归一化值（base_speed_pps / max_pps）喂电机。*/
    const float norm = clampf(s_base_speed_pps / s_cfg.max_wheel_pps, -1.0f, 1.0f);
    motor_set_pair(norm, norm);
}

/* ─── 速度闭环：PPS 目标 → 电机归一化 speed ─── */

static void apply_wheel_speed_loop(float l_target, float r_target, float dt)
{
    l_target = clampf(l_target, -s_cfg.max_wheel_pps, s_cfg.max_wheel_pps);
    r_target = clampf(r_target, -s_cfg.max_wheel_pps, s_cfg.max_wheel_pps);

    pid_set_setpoint(&s_spd_pid_l, l_target);
    pid_set_setpoint(&s_spd_pid_r, r_target);

    const float l_measure = (float)encoder_pps(ENCODER_LEFT);
    const float r_measure = (float)encoder_pps(ENCODER_RIGHT);

    const float l_out = pid_step(&s_spd_pid_l, l_measure, dt);
    const float r_out = pid_step(&s_spd_pid_r, r_measure, dt);

    motor_set_pair(l_out, r_out);
}

/* ─── 主 step ─── */

void motion_step(float dt)
{
    if (dt <= 0.0f) return;

    float l_tgt = 0.0f, r_tgt = 0.0f;

    switch (s_mode) {
    case MOTION_MODE_STOP:
        motor_stop_all();
        return;

    case MOTION_MODE_OPEN_LOOP:
        mode_open_loop();
        return;

    case MOTION_MODE_LINE_FOLLOW:
        mode_line_follow_targets(dt, &l_tgt, &r_tgt);
        break;

    case MOTION_MODE_YAW_HOLD:
        mode_yaw_hold_targets(dt, &l_tgt, &r_tgt);
        break;

    case MOTION_MODE_DIFFERENTIAL:
        mode_differential_targets(&l_tgt, &r_tgt);
        break;
    }

    apply_wheel_speed_loop(l_tgt, r_tgt, dt);
}

void motion_stop(void)
{
    motor_stop_all();
    pid_reset(&s_line_pid);
    pid_reset(&s_yaw_pid);
    pid_reset(&s_spd_pid_l);
    pid_reset(&s_spd_pid_r);
}
