#include "yaw_controller.h"

static float wrap_angle_deg(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

void YawController_Init(yaw_controller_t *controller,
                        const pid_config_t *pid_cfg,
                        float rate_ff_gain)
{
    if ((controller == NULL) || (pid_cfg == NULL)) {
        return;
    }

    Pid_Init(&controller->pid, pid_cfg);
    controller->rate_ff_gain = rate_ff_gain;
    controller->last_yaw_deg = 0.0f;
    controller->continuous_yaw_deg = 0.0f;
    controller->measurement_ready = 0u;
}

void YawController_Reset(yaw_controller_t *controller)
{
    if (controller == NULL) {
        return;
    }

    Pid_Reset(&controller->pid);
    controller->last_yaw_deg = 0.0f;
    controller->continuous_yaw_deg = 0.0f;
    controller->measurement_ready = 0u;
}

void YawController_SetPidConfig(yaw_controller_t *controller, const pid_config_t *pid_cfg)
{
    if ((controller == NULL) || (pid_cfg == NULL)) {
        return;
    }

    Pid_SetConfig(&controller->pid, pid_cfg);
}

float YawController_Update(yaw_controller_t *controller,
                           float target_yaw_deg,
                           float current_yaw_deg,
                           float current_gyro_z,
                           float dt_s)
{
    float rate_ff = 0.0f;
    float yaw_error_deg;
    float wrapped_target_yaw_deg;
    float continuous_yaw_deg;

    if (controller == NULL) {
        return 0.0f;
    }

    yaw_error_deg = wrap_angle_deg(target_yaw_deg - current_yaw_deg);

    if (controller->measurement_ready == 0u) {
        controller->last_yaw_deg = current_yaw_deg;
        controller->continuous_yaw_deg = current_yaw_deg;
        controller->pid.state.prev_measurement = controller->continuous_yaw_deg;
        controller->measurement_ready = 1u;
    } else {
        controller->continuous_yaw_deg +=
            wrap_angle_deg(current_yaw_deg - controller->last_yaw_deg);
        controller->last_yaw_deg = current_yaw_deg;
    }
    continuous_yaw_deg = controller->continuous_yaw_deg;
    wrapped_target_yaw_deg = continuous_yaw_deg + yaw_error_deg;

    if (current_gyro_z > 1e-4f || current_gyro_z < -1e-4f) {
        rate_ff = -current_gyro_z * controller->rate_ff_gain;
    }
    return Pid_Update(&controller->pid,
                      wrapped_target_yaw_deg,
                      continuous_yaw_deg,
                      dt_s) + rate_ff;
}
