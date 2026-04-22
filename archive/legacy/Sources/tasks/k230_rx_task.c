#include "k230_rx_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

K230_Frame_t g_k230_frame;

/*
 * k230_rx_task — 等待 K230_Frame_Wait 返回下一帧，把最新帧发布到 g_k230_frame。
 * 心跳判断由 strategy_task 根据 g_k230_frame.last_rx_tick 自行做。
 *
 * Phase 5 阶段 K230 UART 尚未接真实硬件，此任务会长期阻塞在 K230_Frame_Wait。
 * 不影响其它任务。
 */
void k230_rx_task(void *arg)
{
    (void)arg;
    memset(&g_k230_frame, 0, sizeof(g_k230_frame));
    g_k230_frame.frame_id = 0x02;   /* 初始视为"丢失" */
    K230_UART_Init();

    K230_Frame_t local;
    for (;;) {
        if (K230_Frame_Wait(&local, portMAX_DELAY) == pdTRUE) {
            g_k230_frame = local;
        }
    }
}
