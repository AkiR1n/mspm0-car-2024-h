/*
 * Gimbal.h — 2 轴云台 PID 闭环
 *
 * 输入：归一化图像误差 (x_err, y_err) ∈ [-1, +1]，正右下方向为正
 * 输出：通过 Servo_SetAngle 驱动 pan / tilt
 *
 * 复用 Drivers/Motor_Encoder_PID/pid.c 的 PID_t 结构。
 *
 * 典型用法（在 strategy_task 或 pid_task 里以 10 ms 周期调用）：
 *   static Gimbal_t g;
 *   Gimbal_Init(&g);
 *   while (1) { Gimbal_Update(&g, x_err, y_err); vTaskDelay(10ms); }
 */
#ifndef _GIMBAL_H_
#define _GIMBAL_H_

#include <stdint.h>
#include "pid.h"

typedef struct {
    PID_Controller_t pid_pan;
    PID_Controller_t pid_tilt;

    float pan_angle_deg;        // 当前角度（写回舵机用）
    float tilt_angle_deg;

    float pan_min, pan_max;     // 角度限幅
    float tilt_min, tilt_max;

    float pixel_dead_zone;      // 归一化像素死区，|err| < dz 时不做修正
} Gimbal_t;

void Gimbal_Init(Gimbal_t *g);
void Gimbal_Update(Gimbal_t *g, float x_err_norm, float y_err_norm);
void Gimbal_Reset(Gimbal_t *g);
void Gimbal_SetCenter(Gimbal_t *g);

#endif  /* _GIMBAL_H_ */
