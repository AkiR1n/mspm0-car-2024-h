#include "turn_detection.h"
#include "linetracker.h"
#include <stddef.h>

const turn_detection_config_t TURN_DETECTION_CONFIG_DEFAULT = {
    .threshold_sensors = 2,
    .stable_ms         = 2,
    .timeout_ms        = 100,
    .inhibit_ms        = 800,
};

typedef enum {
    ST_IDLE,
    ST_DETECTING,
    ST_CONFIRMED,
    ST_INHIBITED,
} phase_t;

static turn_detection_config_t s_cfg;
static phase_t                 s_phase[2];   /* 左右各一套 */
static uint32_t                s_detect_start[2];
static uint32_t                s_inhibit_start[2];
static turn_dir_t              s_pending = TURN_DIR_NONE;

void turn_detection_init(const turn_detection_config_t *cfg)
{
    s_cfg = (cfg != NULL) ? *cfg : TURN_DETECTION_CONFIG_DEFAULT;
    s_phase[0] = ST_IDLE;
    s_phase[1] = ST_IDLE;
    s_detect_start[0] = s_detect_start[1] = 0;
    s_inhibit_start[0] = s_inhibit_start[1] = 0;
    s_pending = TURN_DIR_NONE;
}

static uint8_t count_side(bool left)
{
    const linetracker_state_t *st = linetracker_peek();
    /* 左 3 路：index 0, 1, 2；右 3 路：index 4, 5, 6（中间 index 3 共用） */
    const uint8_t a = left ? st->sensor_values[0] : st->sensor_values[6];
    const uint8_t b = left ? st->sensor_values[1] : st->sensor_values[5];
    const uint8_t c = left ? st->sensor_values[2] : st->sensor_values[4];
    return (uint8_t)(a + b + c);
}

static void step_side(bool left, uint32_t now_ms)
{
    const uint8_t idx  = left ? 0u : 1u;
    const turn_dir_t d = left ? TURN_DIR_LEFT : TURN_DIR_RIGHT;

    const uint8_t count = count_side(left);
    const bool    above = (count >= s_cfg.threshold_sensors);

    switch (s_phase[idx]) {
    case ST_IDLE:
        if (above) {
            s_phase[idx]        = ST_DETECTING;
            s_detect_start[idx] = now_ms;
        }
        break;

    case ST_DETECTING: {
        if (!above) {
            s_phase[idx] = ST_IDLE;
            break;
        }
        const uint32_t elapsed = now_ms - s_detect_start[idx];
        if (elapsed >= s_cfg.stable_ms) {
            s_phase[idx]         = ST_CONFIRMED;
            s_inhibit_start[idx] = now_ms;
            if (s_pending == TURN_DIR_NONE) s_pending = d;   /* 只在无待处理事件时发布 */
        } else if (elapsed >= s_cfg.timeout_ms) {
            s_phase[idx] = ST_IDLE;
        }
        break;
    }

    case ST_CONFIRMED:
        /* 已发布事件。等 consume()/外部处理；同时进入抑制。*/
        s_phase[idx] = ST_INHIBITED;
        s_inhibit_start[idx] = now_ms;
        break;

    case ST_INHIBITED: {
        if ((now_ms - s_inhibit_start[idx]) >= s_cfg.inhibit_ms) {
            s_phase[idx] = ST_IDLE;
        }
        break;
    }
    }
}

void turn_detection_update(uint32_t now_ms)
{
    step_side(true,  now_ms);   /* 左 */
    step_side(false, now_ms);   /* 右 */
}

turn_dir_t turn_detection_consume(void)
{
    const turn_dir_t d = s_pending;
    s_pending = TURN_DIR_NONE;
    return d;
}

void turn_detection_reset(void)
{
    s_phase[0] = s_phase[1] = ST_IDLE;
    s_pending = TURN_DIR_NONE;
}
