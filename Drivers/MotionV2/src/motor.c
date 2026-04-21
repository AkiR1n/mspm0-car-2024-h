#include "motor.h"
#include "ti_msp_dl_config.h"
#include <stddef.h>

const motor_config_t MOTOR_CONFIG_DEFAULT = {
    .min_duty      = 0.10f,
    .right_balance = 1.00f,
};

static motor_config_t s_cfg;
static bool           s_pwm_running = false;

static inline float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* ─── HW 操作：方向引脚 + CCP PWM 写入 ─── */

static void apply_direction(motor_id_t id, motor_dir_t dir)
{
    /* 两通道 H 桥真值表：IN1 / IN2
     *   FORWARD: IN1=1, IN2=0
     *   REVERSE: IN1=0, IN2=1
     *   COAST  : IN1=0, IN2=0（两端悬空）
     *   BRAKE  : IN1=1, IN2=1（两端拉低或拉高等效短路）
     *
     * MSPM0 H 桥的物理连线（SysConfig 生成）：
     *   MOTOR_LEFT:  AIN1/AIN2
     *   MOTOR_RIGHT: BIN1/BIN2
     */
    const uint32_t in1_pin = (id == MOTOR_LEFT)
        ? GPIO_MOTOR_PIN_AIN1_PIN : GPIO_MOTOR_PIN_BIN1_PIN;
    const uint32_t in2_pin = (id == MOTOR_LEFT)
        ? GPIO_MOTOR_PIN_AIN2_PIN : GPIO_MOTOR_PIN_BIN2_PIN;

    switch (dir) {
    case MOTOR_DIR_FORWARD:
        DL_GPIO_setPins  (GPIO_MOTOR_PORT, in1_pin);
        DL_GPIO_clearPins(GPIO_MOTOR_PORT, in2_pin);
        break;
    case MOTOR_DIR_REVERSE:
        DL_GPIO_clearPins(GPIO_MOTOR_PORT, in1_pin);
        DL_GPIO_setPins  (GPIO_MOTOR_PORT, in2_pin);
        break;
    case MOTOR_DIR_BRAKE:
        DL_GPIO_setPins  (GPIO_MOTOR_PORT, in1_pin | in2_pin);
        break;
    case MOTOR_DIR_COAST:
    default:
        DL_GPIO_clearPins(GPIO_MOTOR_PORT, in1_pin | in2_pin);
        break;
    }
}

static void apply_duty(motor_id_t id, float duty)
{
    duty = clampf(duty, 0.0f, 1.0f);
    const uint32_t period  = DL_TimerA_getLoadValue(PWM_MOTOR_INST);
    const uint32_t compare = (uint32_t)((float)period * (1.0f - duty));
    const uint32_t cc_idx  = (id == MOTOR_LEFT)
        ? DL_TIMER_CC_0_INDEX : DL_TIMER_CC_1_INDEX;
    DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, compare, cc_idx);
}

/* ─── 公共 API ─── */

void motor_init(const motor_config_t *cfg)
{
    s_cfg = (cfg != NULL) ? *cfg : MOTOR_CONFIG_DEFAULT;
    motor_stop_all();
    if (!s_pwm_running) {
        DL_TimerA_startCounter(PWM_MOTOR_INST);
        s_pwm_running = true;
    }
}

void motor_set_raw(motor_id_t id, motor_dir_t dir, float duty)
{
    if ((unsigned)id >= MOTOR_COUNT) return;
    apply_direction(id, dir);
    if (dir == MOTOR_DIR_FORWARD || dir == MOTOR_DIR_REVERSE) {
        apply_duty(id, duty);
    } else {
        apply_duty(id, 0.0f);
    }
}

void motor_set(motor_id_t id, float speed)
{
    if ((unsigned)id >= MOTOR_COUNT) return;

    /* 右轮平衡 */
    if (id == MOTOR_RIGHT) speed *= s_cfg.right_balance;

    /* 饱和到 [-1, +1] */
    speed = clampf(speed, -1.0f, 1.0f);

    /* 死区处理：严格 0 直接停；介于 (0, min_duty) 抬升到 min_duty */
    const float mag = (speed < 0.0f) ? -speed : speed;
    if (mag < 1e-6f) {
        motor_stop(id);
        return;
    }
    float duty = mag;
    if (duty < s_cfg.min_duty) duty = s_cfg.min_duty;

    motor_set_raw(id,
                  (speed > 0.0f) ? MOTOR_DIR_FORWARD : MOTOR_DIR_REVERSE,
                  duty);
}

void motor_set_pair(float left, float right)
{
    motor_set(MOTOR_LEFT,  left);
    motor_set(MOTOR_RIGHT, right);
}

void motor_stop(motor_id_t id)
{
    motor_set_raw(id, MOTOR_DIR_COAST, 0.0f);
}

void motor_stop_all(void)
{
    motor_stop(MOTOR_LEFT);
    motor_stop(MOTOR_RIGHT);
}

void motor_brake_all(void)
{
    motor_set_raw(MOTOR_LEFT,  MOTOR_DIR_BRAKE, 0.0f);
    motor_set_raw(MOTOR_RIGHT, MOTOR_DIR_BRAKE, 0.0f);
}
