/*
 * linetracker.h — 7 路循迹传感器驱动（v2 重构版）
 *
 * 相对 v1 (LineTracker/linetracker.h/.c) 的改动：
 *   1. 删掉 init 里的 printf —— v1 的 "初始化完成" printf 在 FreeRTOS 下非线程安全。
 *   2. SENSOR_LOGIC_INVERTED 由 #define → linetracker_config_t.logic_inverted 运行时项。
 *   3. 权重由 7 个 #define → linetracker_config_t.weights[7]。默认值与 v1 一致。
 *   4. 位置输出加 EMA 低通滤波（α ∈ (0, 1]），降低循迹传感器抖动。
 *   5. 删除庞大的 LineState 枚举 + 手写位图 switch 分类（LEFT_TURN / SHARP_LEFT 等）。
 *      v1 里那套分类混合了"判方向"和"判偏置"两个语义，并且被 motor_control 用不到。
 *      v2 把状态判断留给上层（motion_control 直接用 position + sensor_bits）。
 *   6. state 指针由 linetracker_read() 返回 const 指针，避免外部直接改全局。
 *   7. 移除"中断版快速读"与"常规读"的 API 二分 —— 合并为单一 linetracker_read()。
 */
#ifndef MOTION_V2_LINETRACKER_H
#define MOTION_V2_LINETRACKER_H

#include <stdbool.h>
#include <stdint.h>

#define LINE_SENSOR_COUNT 7

typedef struct {
    bool    logic_inverted;                        /* true: 高电平=线；false: 低电平=线 */
    int16_t weights[LINE_SENSOR_COUNT];            /* 位置加权，默认 {-30..+30} 等差 */
    float   position_ema_alpha;                    /* 位置 EMA 系数 (0, 1]，1=不滤波 */
} linetracker_config_t;

extern const linetracker_config_t LINETRACKER_CONFIG_DEFAULT;

typedef struct {
    uint8_t  sensor_values[LINE_SENSOR_COUNT]; /* 每路 0/1（已应用 logic_inverted）*/
    uint8_t  sensor_bits;                      /* bit0=sensor0..bit6=sensor6 */
    uint8_t  active_count;                     /* 激活的传感器数量 */
    int16_t  position;                         /* 加权位置（EMA 后） */
    bool     line_detected;                    /* active_count > 0 */
    bool     all_detected;                     /* active_count == LINE_SENSOR_COUNT（起始/结束线）*/
} linetracker_state_t;

/*
 * 初始化：拷贝配置，清零内部状态。
 * GPIO 端口由 SysConfig 生成，这里不做 pin 配置。
 */
void linetracker_init(const linetracker_config_t *cfg);

/*
 * 同步读取所有传感器 + 刷新状态。
 * 返回只读的 state 指针。指针指向的内容会在下次 linetracker_read 后变。
 */
const linetracker_state_t *linetracker_read(void);

/* 不读取、仅取最近一次 linetracker_read 的结果。 */
const linetracker_state_t *linetracker_peek(void);

/* 便捷读取：线位置（等于 state->position，EMA 后值）。 */
int16_t linetracker_position(void);

/* 便捷读取：位图 */
uint8_t linetracker_bits(void);

/* 获取某一路原始值（0/1）。越界返回 0。 */
uint8_t linetracker_sensor(uint8_t index);

#endif /* MOTION_V2_LINETRACKER_H */
