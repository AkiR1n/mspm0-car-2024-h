#include "motor_hal.h"

#include "ti_msp_dl_config.h"

static bool s_pwm_started = false;

void MotorHal_Init(void)
{
    if (!s_pwm_started) {
        DL_TimerA_startCounter(PWM_MOTOR_INST);
        s_pwm_started = true;
    }
}

void MotorHal_SetDirection(motor_hal_id_t id, motor_hal_dir_t dir)
{
    const uint32_t in1 = (id == MOTOR_HAL_LEFT)
        ? GPIO_MOTOR_PIN_AIN1_PIN : GPIO_MOTOR_PIN_BIN1_PIN;
    const uint32_t in2 = (id == MOTOR_HAL_LEFT)
        ? GPIO_MOTOR_PIN_AIN2_PIN : GPIO_MOTOR_PIN_BIN2_PIN;

    switch (dir) {
    case MOTOR_HAL_DIR_FORWARD:
        DL_GPIO_setPins(GPIO_MOTOR_PORT, in1);
        DL_GPIO_clearPins(GPIO_MOTOR_PORT, in2);
        break;
    case MOTOR_HAL_DIR_REVERSE:
        DL_GPIO_clearPins(GPIO_MOTOR_PORT, in1);
        DL_GPIO_setPins(GPIO_MOTOR_PORT, in2);
        break;
    case MOTOR_HAL_DIR_COAST:
    default:
        DL_GPIO_clearPins(GPIO_MOTOR_PORT, in1 | in2);
        break;
    }
}

void MotorHal_SetDuty(motor_hal_id_t id, float duty)
{
    uint32_t compare;
    const uint32_t period = DL_TimerA_getLoadValue(PWM_MOTOR_INST);
    const DL_TIMER_CC_INDEX cc_index = (id == MOTOR_HAL_LEFT)
        ? DL_TIMER_CC_0_INDEX : DL_TIMER_CC_1_INDEX;

    if (duty < 0.0f) {
        duty = 0.0f;
    }
    if (duty > 1.0f) {
        duty = 1.0f;
    }

    compare = (uint32_t)((float)period * (1.0f - duty));
    DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, compare, cc_index);
}
