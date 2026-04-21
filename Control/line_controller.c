#include "line_controller.h"

void LineController_Init(line_controller_t *controller, float kp, float ki, float kd)
{
    if (controller == NULL) {
        return;
    }

    Pid_Init(&controller->pid, kp, ki, kd, -6.0f, 6.0f, -2.0f, 2.0f);
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
