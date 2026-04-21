#ifndef CONTROL_YAW_CONTROLLER_H_
#define CONTROL_YAW_CONTROLLER_H_

#include <stddef.h>

#include "pid.h"

typedef struct {
    pid_t pid;
} yaw_controller_t;

void YawController_Init(yaw_controller_t *controller, float kp, float ki, float kd);
float YawController_Update(yaw_controller_t *controller,
                           float target_yaw_deg,
                           float current_yaw_deg,
                           float current_gyro_z,
                           float dt_s);

#endif
