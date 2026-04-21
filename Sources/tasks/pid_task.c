#include "pid_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "motor_control.h"
#include "k230_rx_task.h"
#include "gimbal.h"

Gimbal_t g_gimbal;

/*
 * pid_task — 10 ms 周期
 *   - 电机差速 PID（调 MotorControl_Update，内部走 line/yaw/speed PID）
 *   - 云台 PID（根据 K230 最近一帧做 pan/tilt 修正）
 *
 * 注意：原 car-25 在 TIMA1_IRQHandler 里跑 MotorControl_Update。
 * 本工程把它搬到任务，周期抖动由 FreeRTOS tick（1 ms）保证。
 */
void pid_task(void *arg)
{
    (void)arg;
    MotorControl_Init();
    Gimbal_Init(&g_gimbal);

    TickType_t next = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10);

    for (;;) {
        MotorControl_Update();

        /* 只有 K230 帧有效且非丢失时才驱动云台。*/
        extern K230_Frame_t g_k230_frame;
        if (g_k230_frame.frame_id == 0x01) {
            Gimbal_Update(&g_gimbal, g_k230_frame.x_norm, g_k230_frame.y_norm);
        }

        vTaskDelayUntil(&next, period);
    }
}
