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

void chassis_system_init(void)
{
    pid_t speed_pid;
    const motor_cfg_t left_motor_cfg = {
        .hal_id = MOTOR_HAL_LEFT,
        .min_duty = 0.10f,
    };
    const motor_cfg_t right_motor_cfg = {
        .hal_id = MOTOR_HAL_RIGHT,
        .min_duty = 0.10f,
    };
    const encoder_cfg_t left_encoder_cfg = {
        .hal_id = ENCODER_HAL_LEFT,
        .pulses_per_revolution = 13u * 28u * 4u,
        .wheel_radius_m = 0.0325f,
        .sample_period_s = 0.01f,
    };
    const encoder_cfg_t right_encoder_cfg = {
        .hal_id = ENCODER_HAL_RIGHT,
        .pulses_per_revolution = 13u * 28u * 4u,
        .wheel_radius_m = 0.0325f,
        .sample_period_s = 0.01f,
    };

    if (s_initialized) {
        return;
    }

    Motor_Init(&s_left_motor, &left_motor_cfg);
    Motor_Init(&s_right_motor, &right_motor_cfg);
    Encoder_Init(&s_left_encoder, &left_encoder_cfg);
    Encoder_Init(&s_right_encoder, &right_encoder_cfg);
    LineSensor_Init(&s_line_sensor);

    Pid_Init(&speed_pid, 6.0f, 18.0f, 0.02f, -1.0f, 1.0f, -0.5f, 0.5f);
    Wheel_Init(&s_left_wheel, &s_left_motor, &s_left_encoder, &speed_pid);
    Wheel_Init(&s_right_wheel, &s_right_motor, &s_right_encoder, &speed_pid);
    Chassis_Init(&s_chassis, &s_left_wheel, &s_right_wheel, 0.14f);
    YawController_Init(&s_yaw_controller, 0.02f, 0.0f, 0.0005f);
    LineController_Init(&s_line_controller, 0.08f, 0.0f, 0.001f);

    s_imu.ready = 0u;
    NVIC_EnableIRQ(TIMER_CALC_INST_INT_IRQN);
    DL_TimerA_startCounter(TIMER_CALC_INST);
    s_initialized = true;
}

chassis_t *chassis_system_get_chassis(void)
{
    return &s_chassis;
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
