#include "wheel.h"

void Wheel_Init(wheel_t *wheel, motor_t *motor, encoder_t *encoder, const pid_t *pid_cfg)
{
    if ((wheel == NULL) || (motor == NULL) || (encoder == NULL) || (pid_cfg == NULL)) {
        return;
    }

    wheel->motor = motor;
    wheel->encoder = encoder;
    wheel->speed_pid = *pid_cfg;
    wheel->target_speed_mps = 0.0f;
    wheel->measured_speed_mps = 0.0f;
    wheel->measured_count = 0;
}

void Wheel_SetTargetSpeed(wheel_t *wheel, float speed_mps)
{
    if (wheel == NULL) {
        return;
    }
    wheel->target_speed_mps = speed_mps;
}

void Wheel_UpdateFeedback(wheel_t *wheel)
{
    if (wheel == NULL) {
        return;
    }

    wheel->measured_speed_mps = Encoder_GetSpeedMps(wheel->encoder);
    wheel->measured_count = Encoder_GetCount(wheel->encoder);
}

void Wheel_ControlStep(wheel_t *wheel, float dt_s)
{
    float duty;

    if (wheel == NULL) {
        return;
    }

    if ((wheel->target_speed_mps > -1e-4f) && (wheel->target_speed_mps < 1e-4f)) {
        Pid_Reset(&wheel->speed_pid);
        Motor_Stop(wheel->motor);
        return;
    }

    duty = Pid_Update(&wheel->speed_pid,
                      wheel->target_speed_mps,
                      wheel->measured_speed_mps,
                      dt_s);
    Motor_SetDuty(wheel->motor, duty);
}

void Wheel_Stop(wheel_t *wheel)
{
    if (wheel == NULL) {
        return;
    }

    wheel->target_speed_mps = 0.0f;
    wheel->measured_speed_mps = 0.0f;
    Pid_Reset(&wheel->speed_pid);
    Motor_Stop(wheel->motor);
}

float Wheel_GetMeasuredSpeed(const wheel_t *wheel)
{
    if (wheel == NULL) {
        return 0.0f;
    }
    return wheel->measured_speed_mps;
}

float Wheel_GetTargetSpeed(const wheel_t *wheel)
{
    if (wheel == NULL) {
        return 0.0f;
    }
    return wheel->target_speed_mps;
}

float Wheel_GetDuty(const wheel_t *wheel)
{
    if (wheel == NULL) {
        return 0.0f;
    }
    return Motor_GetAppliedDuty(wheel->motor);
}
