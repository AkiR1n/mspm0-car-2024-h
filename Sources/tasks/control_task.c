#include "control_task.h"

#include "FreeRTOS.h"
#include "task.h"

#include "app_state.h"
#include "chassis.h"
#include "chassis_system.h"

#define CONTROL_TASK_PERIOD_MS  10U

void control_task(void *arg)
{
    TickType_t next;
    chassis_t *chassis;

    (void)arg;

    chassis_system_init();
    chassis = chassis_system_get_chassis();
    next = xTaskGetTickCount();

    for (;;) {
        chassis_feedback_t feedback;
        chassis_command_t command;
        chassis_debug_t debug;

        app_state_get_feedback(&feedback);
        app_state_get_command(&command);

        if ((command.stop != 0u) || (command.enable_closed_loop == 0u)) {
            Chassis_Stop(chassis);
        } else {
            Chassis_SetTwist(chassis, command.v_mps, command.w_radps);
            Chassis_ControlStep(chassis, (float)CONTROL_TASK_PERIOD_MS / 1000.0f, &debug);
            app_state_set_debug(&debug);
            vTaskDelayUntil(&next, pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
            continue;
        }

        debug.left_target_mps = 0.0f;
        debug.right_target_mps = 0.0f;
        debug.left_measured_mps = feedback.left_speed_mps;
        debug.right_measured_mps = feedback.right_speed_mps;
        debug.left_motor_duty = 0.0f;
        debug.right_motor_duty = 0.0f;
        app_state_set_debug(&debug);
        vTaskDelayUntil(&next, pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
    }
}
