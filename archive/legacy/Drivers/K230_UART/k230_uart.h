/*
 * K230_UART.h — K230 视觉模块 UART 协议驱动
 *
 * 协议（固定 10 字节帧）：
 *   0xAA 0x55  FRAME_ID  X_H X_L  Y_H Y_L  CONF  XOR  0xFF
 *             (1 B)      (2 B)    (2 B)    (1 B) (1 B)
 *
 *   - FRAME_ID: 0x01 目标有效 / 0x02 目标丢失
 *   - X / Y:    int16 归一化坐标，scale = 10000，=> float = /10000
 *   - CONF:     置信度 0–255
 *   - XOR:      前 7 字节异或校验
 *
 * 实现要点：
 *   - UART1 + DMA RX + 空闲中断；ISR 只 xTaskNotifyFromISR 给 k230_rx_task
 *   - 任务消费 DMA buffer，按帧头 AA 55 对齐
 *   - 心跳：last_rx_tick 由任务每帧更新；超时判断放 strategy_task
 *
 * SysConfig 要求（用户在 GUI 里加）：
 *   - 新增 UART 实例，$name = "K230_UART"
 *   - 波特率 115200，使能 DMA RX，使能 UART_IDLE_EVENT
 *   - 对应 DMA 通道 $name = "DMA_K230_CHAN_ID"
 */
#ifndef _K230_UART_H_
#define _K230_UART_H_

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

#define K230_FRAME_LEN       10
#define K230_FRAME_HEAD0     0xAA
#define K230_FRAME_HEAD1     0x55
#define K230_FRAME_TAIL      0xFF

#define K230_ID_VALID        0x01
#define K230_ID_LOST         0x02

#define K230_COORD_SCALE     10000.0f

typedef struct {
    float    x_norm;        /* [-1, +1]，右为正 */
    float    y_norm;        /* [-1, +1]，下为正 */
    uint8_t  conf;
    uint8_t  frame_id;
    uint32_t last_rx_tick;  /* xTaskGetTickCount 值 */
} K230_Frame_t;

void       K230_UART_Init(void);

/*
 * 阻塞等待下一帧解析完成。timeout 可用 portMAX_DELAY。
 * 返回 pdTRUE 表示 out 已填充；pdFALSE 表示超时。
 */
BaseType_t K230_Frame_Wait(K230_Frame_t *out, TickType_t timeout);

/*
 * 非阻塞读取最近一帧快照（不等新帧）。
 */
void       K230_Frame_Peek(K230_Frame_t *out);

#endif  /* _K230_UART_H_ */
