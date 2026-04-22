#include "control_task.h"

#include "FreeRTOS.h"
#include "task.h"

#include "app_state.h"
#include "chassis.h"
#include "chassis_system.h"
#include "motor_drv.h"

#define CONTROL_TASK_PERIOD_MS  10U

void control_task(void *arg)
{
    TickType_t next;
    chassis_t *chassis;
    motor_t *left_motor;
    motor_t *right_motor;

    (void)arg;

    chassis_system_init();
    chassis = chassis_system_get_chassis();
    left_motor = chassis_system_get_left_motor();
    right_motor = chassis_system_get_right_motor();
    next = xTaskGetTickCount();

    for (;;) {
        chassis_feedback_t feedback;
        chassis_command_t command;
        chassis_debug_t debug;

        app_state_get_feedback(&feedback);
        app_state_get_command(&command);

        if ((command.stop != 0u) || (app_state_get_mode() == APP_MODE_STOP)) {
            Chassis_Stop(chassis);
            Motor_Stop(left_motor);
            Motor_Stop(right_motor);
        } else if (app_state_get_mode() == APP_MODE_WHEEL_SPEED_TEST) {
            if (command.enable_closed_loop == 0u) {
                Chassis_Stop(chassis);
                debug.left_target_mps = 0.0f;
                debug.right_target_mps = 0.0f;
                debug.left_measured_mps = feedback.left_speed_mps;
                debug.right_measured_mps = feedback.right_speed_mps;
                debug.left_motor_duty = 0.0f;
                debug.right_motor_duty = 0.0f;
            } else {
                Chassis_SetWheelSpeed(chassis, command.left_speed_mps, command.right_speed_mps);
                Chassis_ControlStep(chassis, (float)CONTROL_TASK_PERIOD_MS / 1000.0f, &debug);
            }
            app_state_set_debug(&debug);
            vTaskDelayUntil(&next, pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
            continue;
        } else if (app_state_get_mode() == APP_MODE_WHEEL_TEST) {
            Motor_SetDuty(left_motor, command.left_duty);
            Motor_SetDuty(right_motor, command.right_duty);
            debug.left_target_mps = command.left_duty;
            debug.right_target_mps = command.right_duty;
            debug.left_measured_mps = feedback.left_speed_mps;
            debug.right_measured_mps = feedback.right_speed_mps;
            debug.left_motor_duty = Motor_GetAppliedDuty(left_motor);
            debug.right_motor_duty = Motor_GetAppliedDuty(right_motor);
            app_state_set_debug(&debug);
            vTaskDelayUntil(&next, pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
            continue;
        } else if (command.enable_closed_loop == 0u) {
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
