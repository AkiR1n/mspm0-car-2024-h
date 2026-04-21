#include "linetracker.h"
#include "ti_msp_dl_config.h"
#include <string.h>
#include <stddef.h>

const linetracker_config_t LINETRACKER_CONFIG_DEFAULT = {
    .logic_inverted     = true,                        /* v1 默认 */
    .weights            = {-30, -20, -10, 0, +10, +20, +30},
    .position_ema_alpha = 0.6f,
};

/* SysConfig 生成的 7 个引脚的 {port, pin} 映射。 */
static const struct {
    GPIO_Regs *port;
    uint32_t   pin;
} s_pins[LINE_SENSOR_COUNT] = {
    {GPIO_TRM_PIN_OUT1_PORT, GPIO_TRM_PIN_OUT1_PIN},   /* 最左 */
    {GPIO_TRM_PIN_OUT2_PORT, GPIO_TRM_PIN_OUT2_PIN},
    {GPIO_TRM_PIN_OUT3_PORT, GPIO_TRM_PIN_OUT3_PIN},
    {GPIO_TRM_PIN_OUT4_PORT, GPIO_TRM_PIN_OUT4_PIN},   /* 中间 */
    {GPIO_TRM_PIN_OUT5_PORT, GPIO_TRM_PIN_OUT5_PIN},
    {GPIO_TRM_PIN_OUT6_PORT, GPIO_TRM_PIN_OUT6_PIN},
    {GPIO_TRM_PIN_OUT7_PORT, GPIO_TRM_PIN_OUT7_PIN},   /* 最右 */
};

static linetracker_config_t s_cfg;
static linetracker_state_t  s_state;
static float                s_position_ema;           /* EMA 累加器（float 避免 int 截断） */

void linetracker_init(const linetracker_config_t *cfg)
{
    s_cfg = (cfg != NULL) ? *cfg : LINETRACKER_CONFIG_DEFAULT;
    memset(&s_state, 0, sizeof(s_state));
    s_position_ema = 0.0f;
}

const linetracker_state_t *linetracker_read(void)
{
    /* ─── 读 7 路 GPIO ─── */
    uint8_t bits  = 0;
    uint8_t count = 0;

    for (uint8_t i = 0; i < LINE_SENSOR_COUNT; ++i) {
        const uint32_t raw = DL_GPIO_readPins(s_pins[i].port, s_pins[i].pin);
        const uint8_t  lvl = (raw != 0) ? 1 : 0;
        const uint8_t  val = s_cfg.logic_inverted ? lvl : (uint8_t)(1u - lvl);

        s_state.sensor_values[i] = val;
        if (val) {
            bits  |= (uint8_t)(1u << i);
            count += 1u;
        }
    }

    s_state.sensor_bits   = bits;
    s_state.active_count  = count;
    s_state.line_detected = (count > 0);
    s_state.all_detected  = (count == LINE_SENSOR_COUNT);

    /* ─── 加权位置 ─── */
    if (count > 0) {
        int32_t sum = 0;
        for (uint8_t i = 0; i < LINE_SENSOR_COUNT; ++i) {
            if (s_state.sensor_values[i]) sum += s_cfg.weights[i];
        }
        const float raw_pos = (float)sum / (float)count;

        /* EMA: new = α·raw + (1-α)·last */
        const float a = s_cfg.position_ema_alpha;
        s_position_ema = a * raw_pos + (1.0f - a) * s_position_ema;
    }
    /* 无线时保持上次 position 不变（不冲到 0，保留最后一次侧偏方向，便于 line_lost 策略）*/

    /* 取整到 int16 */
    s_state.position = (int16_t)((s_position_ema > 0)
                                  ? (s_position_ema + 0.5f)
                                  : (s_position_ema - 0.5f));
    return &s_state;
}

const linetracker_state_t *linetracker_peek(void)
{
    return &s_state;
}

int16_t linetracker_position(void) { return s_state.position; }
uint8_t linetracker_bits(void)     { return s_state.sensor_bits; }

uint8_t linetracker_sensor(uint8_t index)
{
    if (index >= LINE_SENSOR_COUNT) return 0;
    return s_state.sensor_values[index];
}
