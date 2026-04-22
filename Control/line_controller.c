#include "line_controller.h"

void LineController_Init(line_controller_t *controller, const pid_config_t *pid_cfg)
{
    if ((controller == NULL) || (pid_cfg == NULL)) {
        return;
    }

    Pid_Init(&controller->pid, pid_cfg);
}

void LineController_Reset(line_controller_t *controller)
{
    if (controller == NULL) {
        return;
    }

    Pid_Reset(&controller->pid);
}

void LineController_SetPidConfig(line_controller_t *controller, const pid_config_t *pid_cfg)
{
    if ((controller == NULL) || (pid_cfg == NULL)) {
        return;
    }

    Pid_SetConfig(&controller->pid, pid_cfg);
}

float LineController_Update(line_controller_t *controller,
                            float line_error,
                            uint8_t bits,
                            uint8_t detected,
                            float dt_s)
{
    (void)bits;

    if ((controller == NULL) || (detected == 0u)) {
        return 0.0f;
    }

    return Pid_Update(&controller->pid, 0.0f, line_error, dt_s);
}
