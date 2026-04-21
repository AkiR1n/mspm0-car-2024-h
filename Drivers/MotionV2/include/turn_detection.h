/*
 * turn_detection.h — 赛道转弯检测（v2 重构版）
 *
 * 相对 v1 (LineTracker/turn_detection.h/.c) 的改动：
 *   1. 支持左右双向检测（v1 只有左）。用 turn_dir_t 枚举表达。
 *   2. init 不再自动启动 TIMG8 —— v1 的 `NVIC_EnableIRQ + DL_TimerG_startCounter`
 *      和我们现在的 FreeRTOS 任务化架构冲突（任务自己按周期调 update）。
 *   3. `update(now_ms)` 从参数注入当前时间，而不是内部读全局 tick_ms。
 *      让本模块成为纯逻辑，可在主机上不依赖芯片跑单元测试。
 *   4. 改"turn_ready 布尔" → `consume()` 返回 turn_dir_t 并清零，
 *      避免调用方忘记 reset 导致多次触发。
 *   5. 所有参数（阈值、抑制时长、稳定时长、超时）改 config struct，不再 #define。
 */
#ifndef MOTION_V2_TURN_DETECTION_H
#define MOTION_V2_TURN_DETECTION_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    TURN_DIR_NONE  = 0,
    TURN_DIR_LEFT  = 1,
    TURN_DIR_RIGHT = 2,
} turn_dir_t;

typedef struct {
    uint8_t  threshold_sensors; /* 侧边 3 路传感器里至少多少路激活才视为转弯信号。默认 2 */
    uint32_t stable_ms;         /* 信号稳定多久才确认。默认 2 ms */
    uint32_t timeout_ms;        /* 信号从检测到到确认的最大等待。默认 100 ms */
    uint32_t inhibit_ms;        /* 确认后的抑制窗口，防止同一弯连续触发。默认 800 ms */
} turn_detection_config_t;

extern const turn_detection_config_t TURN_DETECTION_CONFIG_DEFAULT;

/*
 * 初始化：拷贝配置，清零内部状态。不启动任何硬件。
 * 调用方典型流程（FreeRTOS 任务）：
 *   turn_detection_init(NULL);
 *   for (;;) {
 *       linetracker_read();                                // 刷新传感器
 *       turn_detection_update(xTaskGetTickCount() * portTICK_PERIOD_MS);
 *       turn_dir_t d = turn_detection_consume();
 *       if (d != TURN_DIR_NONE) ...
 *       vTaskDelay(pdMS_TO_TICKS(5));
 *   }
 */
void       turn_detection_init(const turn_detection_config_t *cfg);

/*
 * 推进状态机一次。需先有最新 linetracker_read() 结果。
 *   now_ms — 当前毫秒时间戳（从 0 起单调递增）。
 */
void       turn_detection_update(uint32_t now_ms);

/*
 * 取一次事件。若有转弯事件，返回方向并清除（一次性 trigger）。
 * 若无事件返回 TURN_DIR_NONE。
 */
turn_dir_t turn_detection_consume(void);

/* 强制清状态（比如手动干预后）。 */
void       turn_detection_reset(void);

#endif /* MOTION_V2_TURN_DETECTION_H */
