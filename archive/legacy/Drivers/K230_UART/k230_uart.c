#include "k230_uart.h"
#include "ti_msp_dl_config.h"
#include <string.h>

/*
 * 实现分两层：
 *
 *   1. 底层 ISR（需要 SysConfig 里定义 K230_UART_INST + 对应 DMA）
 *      → 当帧头 AA 55 + 尾 FF + XOR 校验通过时，解析后写入 s_last，
 *         然后 xTaskNotifyGiveFromISR 唤醒 waiter。
 *
 *   2. 上层 API：K230_Frame_Wait / K230_Frame_Peek 纯软件。
 *
 * 本版先只写上层 API + 解析函数，ISR 留 #ifdef 保护。等 Phase 2.5（SysConfig
 * 加入 K230_UART）之后再取消 #ifdef 并测试。
 */

static K230_Frame_t s_last;
static TaskHandle_t s_waiter = NULL;

static uint8_t xor_sum(const uint8_t *buf, int n)
{
    uint8_t s = 0;
    for (int i = 0; i < n; ++i) s ^= buf[i];
    return s;
}

/*
 * 把 10 字节原始帧解析到 K230_Frame_t；返回 pdTRUE 表示校验通过。
 * 调用方保证 raw 指向连续的 10 字节。
 */
static BaseType_t parse_frame(const uint8_t *raw, K230_Frame_t *f)
{
    if (raw[0] != K230_FRAME_HEAD0 || raw[1] != K230_FRAME_HEAD1 ||
        raw[9] != K230_FRAME_TAIL) {
        return pdFALSE;
    }
    if (xor_sum(raw, 7) != raw[8]) {
        return pdFALSE;
    }

    f->frame_id = raw[2];
    int16_t x_i = (int16_t)((raw[3] << 8) | raw[4]);
    int16_t y_i = (int16_t)((raw[5] << 8) | raw[6]);
    f->x_norm = (float)x_i / K230_COORD_SCALE;
    f->y_norm = (float)y_i / K230_COORD_SCALE;
    f->conf   = raw[7];
    return pdTRUE;
}

void K230_UART_Init(void)
{
    memset(&s_last, 0, sizeof(s_last));
    s_last.frame_id = K230_ID_LOST;
#if defined K230_UART_INST
    /* SysConfig 已配好 UART + DMA；这里做额外的使能和缓冲启动。
     * 具体实现留在 SysConfig 补齐后填充。*/
#endif
}

BaseType_t K230_Frame_Wait(K230_Frame_t *out, TickType_t timeout)
{
    if (!out) return pdFALSE;
    s_waiter = xTaskGetCurrentTaskHandle();
    if (ulTaskNotifyTake(pdTRUE, timeout) == 0) {
        s_waiter = NULL;
        return pdFALSE;
    }
    s_waiter = NULL;
    *out = s_last;
    return pdTRUE;
}

void K230_Frame_Peek(K230_Frame_t *out)
{
    if (out) *out = s_last;
}

/*
 * 外部注入接口：让后续 ISR 或调试 task 把原始 10 B 喂进来，统一走解析逻辑。
 * 这一层让本模块不依赖具体 UART 硬件，便于单测。
 */
BaseType_t K230_FeedRawFromISR(const uint8_t *raw10, BaseType_t *higher_prio_wakeup)
{
    K230_Frame_t tmp;
    if (parse_frame(raw10, &tmp) != pdTRUE) return pdFALSE;
    tmp.last_rx_tick = xTaskGetTickCountFromISR();
    s_last = tmp;
    if (s_waiter) {
        vTaskNotifyGiveFromISR(s_waiter, higher_prio_wakeup);
    }
    return pdTRUE;
}

BaseType_t K230_FeedRaw(const uint8_t *raw10)
{
    K230_Frame_t tmp;
    if (parse_frame(raw10, &tmp) != pdTRUE) return pdFALSE;
    tmp.last_rx_tick = xTaskGetTickCount();
    s_last = tmp;
    if (s_waiter) {
        xTaskNotifyGive(s_waiter);
    }
    return pdTRUE;
}
