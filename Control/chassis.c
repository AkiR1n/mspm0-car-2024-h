#include "chassis.h"

void Chassis_Init(chassis_t *chassis, wheel_t *left_wheel, wheel_t *right_wheel, float wheel_base_m)
{
    if ((chassis == NULL) || (left_wheel == NULL) || (right_wheel == NULL)) {
        return;
    }

    chassis->left_wheel = left_wheel;
    chassis->right_wheel = right_wheel;
    chassis->wheel_base_m = wheel_base_m;
    chassis->target_v_mps = 0.0f;
    chassis->target_w_radps = 0.0f;
}

void Chassis_SetWheelSpeed(chassis_t *chassis, float left_mps, float right_mps)
{
    if (chassis == NULL) {
        return;
    }

    Wheel_SetTargetSpeed(chassis->left_wheel, left_mps);
    Wheel_SetTargetSpeed(chassis->right_wheel, right_mps);
}

void Chassis_SetTwist(chassis_t *chassis, float v_mps, float w_radps)
{
    float half_base;

    if (chassis == NULL) {
        return;
    }

    chassis->target_v_mps = v_mps;
    chassis->target_w_radps = w_radps;
    half_base = chassis->wheel_base_m * 0.5f;

    Chassis_SetWheelSpeed(chassis,
                          v_mps - w_radps * half_base,
                          v_mps + w_radps * half_base);
}

void Chassis_ControlStep(chassis_t *chassis, float dt_s, chassis_debug_t *debug)
{
    if (chassis == NULL) {
        return;
    }

    Wheel_UpdateFeedback(chassis->left_wheel);
    Wheel_UpdateFeedback(chassis->right_wheel);
    Wheel_ControlStep(chassis->left_wheel, dt_s);
    Wheel_ControlStep(chassis->right_wheel, dt_s);

    if (debug != NULL) {
        debug->left_target_mps = Wheel_GetTargetSpeed(chassis->left_wheel);
        debug->right_target_mps = Wheel_GetTargetSpeed(chassis->right_wheel);
        debug->left_measured_mps = Wheel_GetMeasuredSpeed(chassis->left_wheel);
        debug->right_measured_mps = Wheel_GetMeasuredSpeed(chassis->right_wheel);
        debug->left_motor_duty = Wheel_GetDuty(chassis->left_wheel);
        debug->right_motor_duty = Wheel_GetDuty(chassis->right_wheel);
    }
}

void Chassis_Stop(chassis_t *chassis)
{
    if (chassis == NULL) {
        return;
    }

    chassis->target_v_mps = 0.0f;
    chassis->target_w_radps = 0.0f;
    Wheel_Stop(chassis->left_wheel);
    Wheel_Stop(chassis->right_wheel);
}
