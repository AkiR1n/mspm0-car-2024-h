#include "gimbal.h"
#include "servo.h"

/* 初值：参数暂设通用保守值，等有板子后在 Phase 7 整定。*/
#define GIMBAL_PAN_KP    8.0f
#define GIMBAL_PAN_KI    0.05f
#define GIMBAL_PAN_KD    1.2f
#define GIMBAL_PAN_IMAX  30.0f
#define GIMBAL_PAN_OMAX  25.0f   /* 单次最大角度增量 deg */

#define GIMBAL_TILT_KP   6.0f
#define GIMBAL_TILT_KI   0.05f
#define GIMBAL_TILT_KD   1.0f
#define GIMBAL_TILT_IMAX 30.0f
#define GIMBAL_TILT_OMAX 20.0f

#define GIMBAL_PAN_MIN   20.0f
#define GIMBAL_PAN_MAX   160.0f
#define GIMBAL_TILT_MIN  30.0f
#define GIMBAL_TILT_MAX  150.0f

#define GIMBAL_PIXEL_DEADZONE   0.02f  /* 归一化误差 |err| < 2% 不修正 */

void Gimbal_Init(Gimbal_t *g)
{
    Servo_Init();

    PID_Init(&g->pid_pan,  GIMBAL_PAN_KP,  GIMBAL_PAN_KI,  GIMBAL_PAN_KD,
             GIMBAL_PAN_IMAX, GIMBAL_PAN_OMAX);
    PID_Init(&g->pid_tilt, GIMBAL_TILT_KP, GIMBAL_TILT_KI, GIMBAL_TILT_KD,
             GIMBAL_TILT_IMAX, GIMBAL_TILT_OMAX);
    PID_SetTarget(&g->pid_pan,  0.0f);
    PID_SetTarget(&g->pid_tilt, 0.0f);

    g->pan_min  = GIMBAL_PAN_MIN;
    g->pan_max  = GIMBAL_PAN_MAX;
    g->tilt_min = GIMBAL_TILT_MIN;
    g->tilt_max = GIMBAL_TILT_MAX;
    g->pixel_dead_zone = GIMBAL_PIXEL_DEADZONE;

    Gimbal_SetCenter(g);
}

void Gimbal_SetCenter(Gimbal_t *g)
{
    g->pan_angle_deg  = 0.5f * (g->pan_min  + g->pan_max);
    g->tilt_angle_deg = 0.5f * (g->tilt_min + g->tilt_max);
    Servo_SetAngle(SERVO_PAN,  g->pan_angle_deg);
    Servo_SetAngle(SERVO_TILT, g->tilt_angle_deg);
}

void Gimbal_Reset(Gimbal_t *g)
{
    PID_Reset(&g->pid_pan);
    PID_Reset(&g->pid_tilt);
    Gimbal_SetCenter(g);
}

static inline float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void Gimbal_Update(Gimbal_t *g, float x_err, float y_err)
{
    /* 死区内不驱动，避免舵机抖动 */
    if (x_err >  g->pixel_dead_zone || x_err < -g->pixel_dead_zone) {
        float dpan = PID_Calculate(&g->pid_pan, x_err);
        /* 图像右为正 → 追向右 → pan 角减小（取决于舵机安装方向，Phase 7 确认）。
         * 这里默认向右误差为正，输出需要把 pan 减小。*/
        g->pan_angle_deg = clampf(g->pan_angle_deg - dpan, g->pan_min, g->pan_max);
        Servo_SetAngle(SERVO_PAN, g->pan_angle_deg);
    }

    if (y_err >  g->pixel_dead_zone || y_err < -g->pixel_dead_zone) {
        float dtilt = PID_Calculate(&g->pid_tilt, y_err);
        g->tilt_angle_deg = clampf(g->tilt_angle_deg - dtilt, g->tilt_min, g->tilt_max);
        Servo_SetAngle(SERVO_TILT, g->tilt_angle_deg);
    }
}
