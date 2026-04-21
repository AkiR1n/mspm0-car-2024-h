/*
 * encoder.h — 正交编码器驱动 + 速度测量（v2 重构版）
 *
 * 相对 v1 (Motor_Encoder_PID/Encoder.h/.c) 的改动：
 *   1. encoder_id_t 枚举代替 0/1/2 magic number。
 *   2. 编码器参数（PPR / 轮径 / 速度采样周期）改 encoder_config_t 运行时传入，
 *      不再 #define 拼音宏（ENCODE_13X / JIANSUBI / QUADRATURE_MULTIPLIER）。
 *   3. encoder_init 不再自动启动 TIMER_CALC（v1 这一行 DL_TimerA_startCounter 会隐式
 *      启动 10 ms PID 定时器，和我们 FreeRTOS 任务化架构冲突）。改由调用方显式开。
 *   4. ISR 钩子更名：Encoder_IRQHandler → encoder_on_gpio_irq；
 *                    Encoder_Timer_IRQHandler → encoder_on_tick_irq。
 *      更明确"这是被 interrupt.c 调用的 handler"。
 *   5. 新增 encoder_mps()（米/秒），比 PPS / RPS 对上层更友好。
 *   6. 删除 v1 的 _Abs 变体（调用方 fabsf 即可）。
 *   7. encoder_on_gpio_irq 不再内部 check GROUP1（上层 GROUP1_IRQHandler 已做）。
 */
#ifndef MOTION_V2_ENCODER_H
#define MOTION_V2_ENCODER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    ENCODER_LEFT  = 0,
    ENCODER_RIGHT = 1,
    ENCODER_COUNT = 2,
} encoder_id_t;

typedef struct {
    /*
     * 编码器每转脉冲数（轮轴侧）= 编码器线数 × 减速比 × 四倍频
     * v1 默认：13 × 28 × 4 = 1456。
     */
    uint32_t pulses_per_revolution;

    /* 车轮半径（毫米）。v1 默认：32.5。用于 encoder_mps() 计算。 */
    float    wheel_radius_mm;

    /*
     * 速度采样周期（毫秒）。encoder_on_tick_irq 的调用频率必须匹配此值。
     * v1 默认：10。
     */
    uint32_t speed_sample_period_ms;
} encoder_config_t;

extern const encoder_config_t ENCODER_CONFIG_DEFAULT;

/*
 * 初始化：清零计数器、配置中断使能。不启动硬件定时器（留给调用方）。
 * 调用方典型流程：
 *   encoder_init(&cfg);
 *   DL_TimerA_startCounter(TIMER_CALC_INST);   // 若还要用 10ms 速度计算
 *   NVIC_EnableIRQ(TIMER_CALC_INST_INT_IRQN);
 */
void encoder_init(const encoder_config_t *cfg);

/* 读取累计计数值（带方向）。 */
int32_t encoder_count(encoder_id_t id);

/* 读取瞬时速度（脉冲/秒），正负表示方向。 */
int32_t encoder_pps(encoder_id_t id);

/* 读取瞬时转速（转/秒），正负表示方向。 */
float   encoder_rps(encoder_id_t id);

/* 读取线速度（米/秒），正负表示方向。 */
float   encoder_mps(encoder_id_t id);

/* 清零指定编码器 / 全部编码器的累计计数和速度。 */
void    encoder_reset(encoder_id_t id);
void    encoder_reset_all(void);

/*
 * GPIO 中断钩子：由 GROUP1_IRQHandler 在判定到编码器引脚中断后调用。
 * 本函数只处理自己相关的 4 个引脚，不再做上层 group 判定。
 */
void    encoder_on_gpio_irq(void);

/*
 * 速度采样周期中断钩子：由 TIMER_CALC 中断服务程序调用，用于周期性计算 PPS。
 * 调用频率必须与 encoder_config_t.speed_sample_period_ms 匹配。
 */
void    encoder_on_tick_irq(void);

#endif /* MOTION_V2_ENCODER_H */
