#include "yaw_controller.h"

void YawController_Init(yaw_controller_t *controller,
                        const pid_config_t *pid_cfg,
                        float rate_ff_gain)
{
    if ((controller == NULL) || (pid_cfg == NULL)) {
        return;
    }

    Pid_Init(&controller->pid, pid_cfg);
    controller->rate_ff_gain = rate_ff_gain;
}

void YawController_Reset(yaw_controller_t *controller)
{
    if (controller == NULL) {
        return;
    }

    Pid_Reset(&controller->pid);
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

    if (controller == NULL) {
        return 0.0f;
    }

    if (current_gyro_z > 1e-4f || current_gyro_z < -1e-4f) {
        rate_ff = -current_gyro_z * controller->rate_ff_gain;
    }
    return Pid_Update(&controller->pid, target_yaw_deg, current_yaw_deg, dt_s) + rate_ff;
}
