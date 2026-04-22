#include "wheel.h"

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

void Wheel_Init(wheel_t *wheel,
                motor_t *motor,
                encoder_t *encoder,
                const wheel_cfg_t *cfg)
{
    if ((wheel == NULL) || (motor == NULL) || (encoder == NULL) || (cfg == NULL)) {
        return;
    }

    wheel->motor = motor;
    wheel->encoder = encoder;
    wheel->cfg = *cfg;
    Pid_Init(&wheel->speed_pid, &cfg->speed_pid);
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
    float pid_output;
    float feedforward;
    float duty;

    if (wheel == NULL) {
        return;
    }

    if ((wheel->target_speed_mps > -1e-4f) && (wheel->target_speed_mps < 1e-4f)) {
        Pid_Reset(&wheel->speed_pid);
        Motor_Stop(wheel->motor);
        return;
    }

    pid_output = Pid_Update(&wheel->speed_pid,
                            wheel->target_speed_mps,
                            wheel->measured_speed_mps,
                            dt_s);
    feedforward = wheel->cfg.speed_ff_gain * wheel->target_speed_mps;
    duty = clampf((pid_output + feedforward) * wheel->cfg.duty_polarity, -1.0f, 1.0f);
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

void Wheel_SetPidConfig(wheel_t *wheel, const pid_config_t *pid_cfg)
{
    if ((wheel == NULL) || (pid_cfg == NULL)) {
        return;
    }

    wheel->cfg.speed_pid = *pid_cfg;
    Pid_SetConfig(&wheel->speed_pid, pid_cfg);
    Pid_Reset(&wheel->speed_pid);
}

void Wheel_GetPidConfig(const wheel_t *wheel, pid_config_t *pid_cfg)
{
    if ((wheel == NULL) || (pid_cfg == NULL)) {
        return;
    }

    Pid_GetConfig(&wheel->speed_pid, pid_cfg);
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
