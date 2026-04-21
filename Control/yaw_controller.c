#include "yaw_controller.h"

void YawController_Init(yaw_controller_t *controller, float kp, float ki, float kd)
{
    if (controller == NULL) {
        return;
    }

    Pid_Init(&controller->pid, kp, ki, kd, -6.0f, 6.0f, -2.0f, 2.0f);
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
        rate_ff = -current_gyro_z * 0.0005f;
    }
    return Pid_Update(&controller->pid, target_yaw_deg, current_yaw_deg, dt_s) + rate_ff;
}
