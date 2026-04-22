#ifndef CONTROL_WHEEL_H_
#define CONTROL_WHEEL_H_

#include "encoder_drv.h"
#include "motor_drv.h"
#include "pid.h"

typedef struct {
    pid_config_t speed_pid;
    float        speed_ff_gain;
} wheel_cfg_t;

typedef struct {
    motor_t          *motor;
    encoder_t        *encoder;
    pid_controller_t speed_pid;
    wheel_cfg_t      cfg;
    float            target_speed_mps;
    float            measured_speed_mps;
    int32_t          measured_count;
} wheel_t;

void Wheel_Init(wheel_t *wheel,
                motor_t *motor,
                encoder_t *encoder,
                const wheel_cfg_t *cfg);
void Wheel_SetTargetSpeed(wheel_t *wheel, float speed_mps);
void Wheel_UpdateFeedback(wheel_t *wheel);
void Wheel_ControlStep(wheel_t *wheel, float dt_s);
void Wheel_Stop(wheel_t *wheel);
void Wheel_SetPidConfig(wheel_t *wheel, const pid_config_t *pid_cfg);
void Wheel_GetPidConfig(const wheel_t *wheel, pid_config_t *pid_cfg);
float Wheel_GetMeasuredSpeed(const wheel_t *wheel);
float Wheel_GetTargetSpeed(const wheel_t *wheel);
float Wheel_GetDuty(const wheel_t *wheel);

#endif
