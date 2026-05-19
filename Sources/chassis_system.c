#include "chassis_system.h"

#include <stdbool.h>

#include "ti_msp_dl_config.h"

static bool s_initialized = false;

static motor_t s_left_motor;
static motor_t s_right_motor;
static encoder_t s_left_encoder;
static encoder_t s_right_encoder;
static wheel_t s_left_wheel;
static wheel_t s_right_wheel;
static chassis_t s_chassis;
static imu_t s_imu;
static line_sensor_t s_line_sensor;
static yaw_controller_t s_yaw_controller;
static line_controller_t s_line_controller;

static wheel_t *chassis_system_get_wheel_by_index(int wheel_index)
{
    if (wheel_index == 0) {
        return &s_left_wheel;
    }
    if (wheel_index == 1) {
        return &s_right_wheel;
    }
    return NULL;
}

void chassis_system_init(void)
{
    const wheel_cfg_t left_wheel_cfg = {
        .speed_pid = {
            .kp = 0.265742f,
            .ki = 3.321770f,
            .kd = 0.0f,
            .out_min = -1.0f,
            .out_max = 1.0f,
            .integral_min = -0.5f,
            .integral_max = 0.5f,
            .d_filter_alpha = 0.85f,
            .anti_windup_enable = 1u,
            .mode = PID_MODE_POSITIONAL,
        },
        .speed_ff_gain = 1.162620f,
    };
    const wheel_cfg_t right_wheel_cfg = {
        .speed_pid = {
            .kp = 0.265832f,
            .ki = 3.322895f,
            .kd = 0.0f,
            .out_min = -1.0f,
            .out_max = 1.0f,
            .integral_min = -0.5f,
            .integral_max = 0.5f,
            .d_filter_alpha = 0.85f,
            .anti_windup_enable = 1u,
            .mode = PID_MODE_POSITIONAL,
        },
        .speed_ff_gain = 1.163013f,
    };
    const pid_config_t yaw_pid_cfg = {
        .kp = 0.02f,
        .ki = 0.0f,
        .kd = 0.0005f,
        .out_min = -6.0f,
        .out_max = 6.0f,
        .integral_min = -2.0f,
        .integral_max = 2.0f,
        .d_filter_alpha = 0.90f,
        .anti_windup_enable = 1u,
        .mode = PID_MODE_POSITIONAL,
    };
    const pid_config_t line_pid_cfg = {
        .kp = 0.067f,
        .ki = 0.0f,
        .kd = 0.0004f,
        .out_min = -2.6f,
        .out_max = 2.6f,
        .integral_min = -2.0f,
        .integral_max = 2.0f,
        .d_filter_alpha = 0.92f,
        .anti_windup_enable = 1u,
        .mode = PID_MODE_POSITIONAL,
    };
    const motor_cfg_t left_motor_cfg = {
        .hal_id = MOTOR_HAL_LEFT,
        .min_duty = 0.10f,
        .direction_sign = -1.0f,
    };
    const motor_cfg_t right_motor_cfg = {
        .hal_id = MOTOR_HAL_RIGHT,
        .min_duty = 0.10f,
        .direction_sign = -1.0f,
    };
    const encoder_cfg_t left_encoder_cfg = {
        .hal_id = ENCODER_HAL_LEFT,
        .pulses_per_revolution = 13u * 28u * 4u,
        .wheel_radius_m = 0.0325f,
        .sample_period_s = 0.01f,
        .direction_sign = 1.0f,
    };
    const encoder_cfg_t right_encoder_cfg = {
        .hal_id = ENCODER_HAL_RIGHT,
        .pulses_per_revolution = 13u * 28u * 4u,
        .wheel_radius_m = 0.0325f,
        .sample_period_s = 0.01f,
        .direction_sign = -1.0f,
    };

    if (s_initialized) {
        return;
    }

    Motor_Init(&s_left_motor, &left_motor_cfg);
    Motor_Init(&s_right_motor, &right_motor_cfg);
    Encoder_Init(&s_left_encoder, &left_encoder_cfg);
    Encoder_Init(&s_right_encoder, &right_encoder_cfg);
    LineSensor_Init(&s_line_sensor);

    Wheel_Init(&s_left_wheel, &s_left_motor, &s_left_encoder, &left_wheel_cfg);
    Wheel_Init(&s_right_wheel, &s_right_motor, &s_right_encoder, &right_wheel_cfg);
    Chassis_Init(&s_chassis, &s_left_wheel, &s_right_wheel, 0.14f);
    YawController_Init(&s_yaw_controller, &yaw_pid_cfg, 0.0005f);
    LineController_Init(&s_line_controller, &line_pid_cfg);

    s_imu.ready = 0u;
    NVIC_EnableIRQ(TIMER_CALC_INST_INT_IRQN);
    DL_TimerA_startCounter(TIMER_CALC_INST);
    s_initialized = true;
}

chassis_t *chassis_system_get_chassis(void)
{
    return &s_chassis;
}

wheel_t *chassis_system_get_left_wheel(void)
{
    return &s_left_wheel;
}

wheel_t *chassis_system_get_right_wheel(void)
{
    return &s_right_wheel;
}

motor_t *chassis_system_get_left_motor(void)
{
    return &s_left_motor;
}

motor_t *chassis_system_get_right_motor(void)
{
    return &s_right_motor;
}

encoder_t *chassis_system_get_left_encoder(void)
{
    return &s_left_encoder;
}

encoder_t *chassis_system_get_right_encoder(void)
{
    return &s_right_encoder;
}

imu_t *chassis_system_get_imu(void)
{
    return &s_imu;
}

line_sensor_t *chassis_system_get_line_sensor(void)
{
    return &s_line_sensor;
}

yaw_controller_t *chassis_system_get_yaw_controller(void)
{
    return &s_yaw_controller;
}

line_controller_t *chassis_system_get_line_controller(void)
{
    return &s_line_controller;
}

void chassis_system_set_wheel_pid_config(int wheel_index, const pid_config_t *pid_cfg)
{
    wheel_t *wheel = chassis_system_get_wheel_by_index(wheel_index);

    if ((wheel == NULL) || (pid_cfg == NULL)) {
        return;
    }

    Wheel_SetPidConfig(wheel, pid_cfg);
}

void chassis_system_get_wheel_pid_config(int wheel_index, pid_config_t *pid_cfg)
{
    wheel_t *wheel = chassis_system_get_wheel_by_index(wheel_index);

    if ((wheel == NULL) || (pid_cfg == NULL)) {
        return;
    }

    Wheel_GetPidConfig(wheel, pid_cfg);
}

void chassis_system_set_wheel_ff_gain(int wheel_index, float ff_gain)
{
    wheel_t *wheel = chassis_system_get_wheel_by_index(wheel_index);

    if (wheel == NULL) {
        return;
    }

    wheel->cfg.speed_ff_gain = ff_gain;
}

float chassis_system_get_wheel_ff_gain(int wheel_index)
{
    wheel_t *wheel = chassis_system_get_wheel_by_index(wheel_index);

    if (wheel == NULL) {
        return 0.0f;
    }

    return wheel->cfg.speed_ff_gain;
}
