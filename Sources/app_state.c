#include "app_state.h"

#include <stddef.h>

#include "FreeRTOS.h"
#include "task.h"

static app_state_snapshot_t s_state;

void app_state_init(void)
{
    taskENTER_CRITICAL();
    s_state.mode = APP_MODE_STOP;
    s_state.command.stop = 1u;
    s_state.command.enable_closed_loop = 1u;
    s_state.command.v_mps = 0.0f;
    s_state.command.w_radps = 0.0f;
    s_state.command.left_duty = 0.0f;
    s_state.command.right_duty = 0.0f;
    s_state.command.left_speed_mps = 0.0f;
    s_state.command.right_speed_mps = 0.0f;
    taskEXIT_CRITICAL();
}

void app_state_set_mode(app_mode_t mode)
{
    taskENTER_CRITICAL();
    s_state.mode = mode;
    taskEXIT_CRITICAL();
}

app_mode_t app_state_get_mode(void)
{
    app_mode_t mode;

    taskENTER_CRITICAL();
    mode = s_state.mode;
    taskEXIT_CRITICAL();
    return mode;
}

void app_state_set_feedback(const chassis_feedback_t *feedback)
{
    if (feedback == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    s_state.feedback = *feedback;
    taskEXIT_CRITICAL();
}

void app_state_get_feedback(chassis_feedback_t *feedback)
{
    if (feedback == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    *feedback = s_state.feedback;
    taskEXIT_CRITICAL();
}

void app_state_set_command(const chassis_command_t *command)
{
    if (command == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    s_state.command = *command;
    taskEXIT_CRITICAL();
}

void app_state_get_command(chassis_command_t *command)
{
    if (command == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    *command = s_state.command;
    taskEXIT_CRITICAL();
}

void app_state_set_debug(const chassis_debug_t *debug)
{
    if (debug == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    s_state.debug = *debug;
    taskEXIT_CRITICAL();
}

void app_state_get_debug(chassis_debug_t *debug)
{
    if (debug == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    *debug = s_state.debug;
    taskEXIT_CRITICAL();
}

void app_state_get_snapshot(app_state_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    *snapshot = s_state;
    taskEXIT_CRITICAL();
}

const char *app_mode_name(app_mode_t mode)
{
    switch (mode) {
    case APP_MODE_WHEEL_SPEED_TEST:
        return "WSPD";
    case APP_MODE_WHEEL_TEST:
        return "WHEEL";
    case APP_MODE_TWIST_OPEN:
        return "TWIST";
    case APP_MODE_STOP:
    default:
        return "STOP";
    }
}
