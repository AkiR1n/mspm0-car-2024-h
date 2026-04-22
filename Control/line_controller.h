#ifndef CONTROL_LINE_CONTROLLER_H_
#define CONTROL_LINE_CONTROLLER_H_

#include <stddef.h>
#include <stdint.h>

#include "pid.h"

typedef struct {
    pid_controller_t pid;
} line_controller_t;

void LineController_Init(line_controller_t *controller, const pid_config_t *pid_cfg);
void LineController_SetPidConfig(line_controller_t *controller, const pid_config_t *pid_cfg);
float LineController_Update(line_controller_t *controller,
                            float line_error,
                            uint8_t bits,
                            uint8_t detected,
                            float dt_s);

#endif
