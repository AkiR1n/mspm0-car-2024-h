#include "strategy_task.h"
#include "k230_rx_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "motor_control.h"

/*
 * strategy_task — 20 ms 周期
 *   - 根据传感器与 K230 心跳状态切换电机模式
 *   - 心跳丢失超过 300 ms 视为视觉"丢目标"，切到搜索（这里先设为 STOP 占位）
 *
 * 当前版本仅做最小骨架，等校赛题目出来后再扩展状态机。
 */
#define K230_HEARTBEAT_TIMEOUT_MS   300

void strategy_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(500));  /* 等 MotorControl 初始化完成 */

    /* 默认：按键控制圈数的循迹模式（对齐 car-25 的主行为）。
     * 后续根据题目切换。*/
    MotorControl_SetMode(MOTOR_MODE_LINE_FOLLOWING);
    MotorControl_SetBaseSpeed(20.0f);

    TickType_t next = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20);

    for (;;) {
        uint32_t now = xTaskGetTickCount();
        uint32_t dt = (now - g_k230_frame.last_rx_tick) * portTICK_PERIOD_MS;
        if (g_k230_frame.last_rx_tick == 0 || dt > K230_HEARTBEAT_TIMEOUT_MS) {
            /* K230 心跳丢：继续循迹但 GIMBAL 不再追踪（pid_task 里靠 frame_id 守护） */
            g_k230_frame.frame_id = 0x02;
        }
        vTaskDelayUntil(&next, period);
    }
}
