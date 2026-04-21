#ifndef CONTROL_CHASSIS_H_
#define CONTROL_CHASSIS_H_

#include <stddef.h>

#include "app_state.h"
#include "wheel.h"

typedef struct {
    wheel_t *left_wheel;
    wheel_t *right_wheel;
    float    wheel_base_m;
    float    target_v_mps;
    float    target_w_radps;
} chassis_t;

void Chassis_Init(chassis_t *chassis, wheel_t *left_wheel, wheel_t *right_wheel, float wheel_base_m);
void Chassis_SetTwist(chassis_t *chassis, float v_mps, float w_radps);
void Chassis_SetWheelSpeed(chassis_t *chassis, float left_mps, float right_mps);
void Chassis_ControlStep(chassis_t *chassis, float dt_s, chassis_debug_t *debug);
void Chassis_Stop(chassis_t *chassis);

#endif
