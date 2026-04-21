#include "motor_drv.h"

static float clampf(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

void Motor_Init(motor_t *motor, const motor_cfg_t *cfg)
{
    if ((motor == NULL) || (cfg == NULL)) {
        return;
    }

    MotorHal_Init();
    motor->cfg = *cfg;
    motor->applied_duty = 0.0f;
    Motor_Stop(motor);
}

void Motor_SetDuty(motor_t *motor, float duty)
{
    float magnitude;

    if (motor == NULL) {
        return;
    }

    duty = clampf(duty, -1.0f, 1.0f);
    magnitude = (duty >= 0.0f) ? duty : -duty;

    if (magnitude < 1e-6f) {
        Motor_Stop(motor);
        return;
    }

    if (magnitude < motor->cfg.min_duty) {
        magnitude = motor->cfg.min_duty;
    }

    MotorHal_SetDirection(motor->cfg.hal_id,
                          (duty >= 0.0f) ? MOTOR_HAL_DIR_FORWARD
                                         : MOTOR_HAL_DIR_REVERSE);
    MotorHal_SetDuty(motor->cfg.hal_id, magnitude);
    motor->applied_duty = (duty >= 0.0f) ? magnitude : -magnitude;
}

void Motor_Stop(motor_t *motor)
{
    if (motor == NULL) {
        return;
    }

    MotorHal_SetDirection(motor->cfg.hal_id, MOTOR_HAL_DIR_COAST);
    MotorHal_SetDuty(motor->cfg.hal_id, 0.0f);
    motor->applied_duty = 0.0f;
}

float Motor_GetAppliedDuty(const motor_t *motor)
{
    if (motor == NULL) {
        return 0.0f;
    }
    return motor->applied_duty;
}
