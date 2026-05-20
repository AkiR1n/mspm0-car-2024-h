#ifndef CONTROL_YAW_CONTROLLER_H_
#define CONTROL_YAW_CONTROLLER_H_

#include <stddef.h>

#include "pid.h"

typedef struct {
    pid_controller_t pid;
    float            rate_ff_gain;
    float            last_yaw_deg;
    float            continuous_yaw_deg;
    unsigned         measurement_ready : 1;
} yaw_controller_t;

void YawController_Init(yaw_controller_t *controller,
                        const pid_config_t *pid_cfg,
                        float rate_ff_gain);
void YawController_Reset(yaw_controller_t *controller);
void YawController_SetPidConfig(yaw_controller_t *controller, const pid_config_t *pid_cfg);
float YawController_Update(yaw_controller_t *controller,
                           float target_yaw_deg,
                           float current_yaw_deg,
                           float current_gyro_z,
                           float dt_s);

#endif
