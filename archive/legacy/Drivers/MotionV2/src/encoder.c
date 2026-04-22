#include "encoder.h"
#include "ti_msp_dl_config.h"
#include <stddef.h>

const encoder_config_t ENCODER_CONFIG_DEFAULT = {
    .pulses_per_revolution  = 13u * 28u * 4u,   /* 线数 × 减速比 × 四倍频 */
    .wheel_radius_mm        = 32.5f,
    .speed_sample_period_ms = 10u,
};

static encoder_config_t s_cfg;

/* ─── ISR 访问的状态：volatile ─── */
static volatile int32_t s_count[ENCODER_COUNT];
static volatile int32_t s_count_last[ENCODER_COUNT];
static volatile int32_t s_pps[ENCODER_COUNT];

/* 从 SysConfig 引脚宏派生的读"1/0"值。保留原项目的"按 active-low 反转"语义。 */
#define READ_BIT(port, pin) \
    ((DL_GPIO_readPins((port), (pin)) == (pin)) ? 0 : 1)

#define READ_A1 READ_BIT(GPIO_ENCODER_PORT, GPIO_ENCODER_PIN_A1_PIN)
#define READ_A2 READ_BIT(GPIO_ENCODER_PORT, GPIO_ENCODER_PIN_A2_PIN)
#define READ_B1 READ_BIT(GPIO_ENCODER_PORT, GPIO_ENCODER_PIN_B1_PIN)
#define READ_B2 READ_BIT(GPIO_ENCODER_PORT, GPIO_ENCODER_PIN_B2_PIN)

void encoder_init(const encoder_config_t *cfg)
{
    s_cfg = (cfg != NULL) ? *cfg : ENCODER_CONFIG_DEFAULT;
    for (int i = 0; i < ENCODER_COUNT; ++i) {
        s_count[i]      = 0;
        s_count_last[i] = 0;
        s_pps[i]        = 0;
    }

    /* 清一次挂起的中断 + 使能 NVIC GPIO 通道。
     * 不启动 TIMER_CALC —— 调用方按场景自己开（任务轮询 or ISR 驱动）。*/
    DL_GPIO_clearInterruptStatus(GPIO_ENCODER_PORT,
        GPIO_ENCODER_PIN_A1_PIN | GPIO_ENCODER_PIN_A2_PIN |
        GPIO_ENCODER_PIN_B1_PIN | GPIO_ENCODER_PIN_B2_PIN);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
}

int32_t encoder_count(encoder_id_t id)
{
    if ((unsigned)id >= ENCODER_COUNT) return 0;
    return s_count[id];
}

int32_t encoder_pps(encoder_id_t id)
{
    if ((unsigned)id >= ENCODER_COUNT) return 0;
    return s_pps[id];
}

float encoder_rps(encoder_id_t id)
{
    if ((unsigned)id >= ENCODER_COUNT) return 0.0f;
    return (float)s_pps[id] / (float)s_cfg.pulses_per_revolution;
}

float encoder_mps(encoder_id_t id)
{
    if ((unsigned)id >= ENCODER_COUNT) return 0.0f;
    const float circumference_m = 2.0f * 3.14159265f * s_cfg.wheel_radius_mm / 1000.0f;
    return encoder_rps(id) * circumference_m;
}

void encoder_reset(encoder_id_t id)
{
    if ((unsigned)id >= ENCODER_COUNT) return;
    s_count[id]      = 0;
    s_count_last[id] = 0;
    s_pps[id]        = 0;
}

void encoder_reset_all(void)
{
    for (int i = 0; i < ENCODER_COUNT; ++i) encoder_reset((encoder_id_t)i);
}

/* ─── GPIO 中断处理 ─── */

void encoder_on_gpio_irq(void)
{
    /*
     * 上层 GROUP1_IRQHandler 已经判定到这是 GPIOB 的中断；这里只处理自己的 4 个引脚。
     * 每个引脚的沿（rising+falling）都会触发 → 四倍频正交解码。
     */
    const uint32_t pins =
        GPIO_ENCODER_PIN_A1_PIN | GPIO_ENCODER_PIN_A2_PIN |
        GPIO_ENCODER_PIN_B1_PIN | GPIO_ENCODER_PIN_B2_PIN;

    const uint32_t st = DL_GPIO_getEnabledInterruptStatus(GPIO_ENCODER_PORT, pins);

    if (st & GPIO_ENCODER_PIN_A1_PIN) {
        DL_GPIO_clearInterruptStatus(GPIO_ENCODER_PORT, GPIO_ENCODER_PIN_A1_PIN);
        s_count[ENCODER_LEFT] += (READ_A1 ^ READ_A2) ? +1 : -1;
    }
    if (st & GPIO_ENCODER_PIN_A2_PIN) {
        DL_GPIO_clearInterruptStatus(GPIO_ENCODER_PORT, GPIO_ENCODER_PIN_A2_PIN);
        s_count[ENCODER_LEFT] += (READ_A1 ^ READ_A2) ? -1 : +1;
    }
    if (st & GPIO_ENCODER_PIN_B1_PIN) {
        DL_GPIO_clearInterruptStatus(GPIO_ENCODER_PORT, GPIO_ENCODER_PIN_B1_PIN);
        s_count[ENCODER_RIGHT] += (READ_B1 ^ READ_B2) ? +1 : -1;
    }
    if (st & GPIO_ENCODER_PIN_B2_PIN) {
        DL_GPIO_clearInterruptStatus(GPIO_ENCODER_PORT, GPIO_ENCODER_PIN_B2_PIN);
        s_count[ENCODER_RIGHT] += (READ_B1 ^ READ_B2) ? -1 : +1;
    }
}

/* ─── 速度采样周期中断 ─── */

void encoder_on_tick_irq(void)
{
    /*
     * PPS = Δcount / Δt。Δt = s_cfg.speed_sample_period_ms / 1000 秒。
     * 换算系数：PPS = Δcount × (1000 / period_ms)。
     */
    const int32_t scale = (int32_t)(1000u / (s_cfg.speed_sample_period_ms == 0u
                                                ? 1u
                                                : s_cfg.speed_sample_period_ms));
    for (int i = 0; i < ENCODER_COUNT; ++i) {
        const int32_t diff = s_count[i] - s_count_last[i];
        s_pps[i]        = diff * scale;
        s_count_last[i] = s_count[i];
    }
}
