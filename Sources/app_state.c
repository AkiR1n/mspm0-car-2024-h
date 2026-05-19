#include "app_state.h"

#include <stddef.h>

#include "FreeRTOS.h"
#include "task.h"
#include "signal_driver.h"

static app_state_snapshot_t s_state;

uint8_t app_challenge_lap_total(app_challenge_t challenge)
{
    if (challenge == APP_CHALLENGE_Q4) {
        return 4u;
    }
    if ((challenge == APP_CHALLENGE_Q1) ||
        (challenge == APP_CHALLENGE_Q2) ||
        (challenge == APP_CHALLENGE_Q3)) {
        return 1u;
    }
    return 0u;
}

void app_state_init(void)
{
    taskENTER_CRITICAL();
    s_state.mode = APP_MODE_STOP;
    s_state.main_state = APP_MAIN_STATE_IDLE;
    s_state.challenge.selected = APP_CHALLENGE_Q1;
    s_state.challenge.active = APP_CHALLENGE_NONE;
    s_state.challenge.status = APP_CHALLENGE_STATUS_READY;
    s_state.challenge.phase = APP_CHALLENGE_PHASE_IDLE;
    s_state.challenge.action = APP_PHASE_ACTION_NONE;
    s_state.challenge.checkpoint_count = 0u;
    s_state.challenge.lap_index = 1u;
    s_state.challenge.lap_total = app_challenge_lap_total(APP_CHALLENGE_Q1);
    s_state.challenge.geometry_heading_deg = 0.0f;
    s_state.challenge.hold_heading_deg = 0.0f;
    s_state.challenge.heading_error_deg = 0.0f;
    s_state.challenge.phase_distance_m = 0.0f;
    s_state.challenge.target_distance_m = 0.0f;
    s_state.challenge.target_speed_mps = 0.0f;
    s_state.challenge.last_event = APP_EVENT_NONE;
    s_state.challenge.last_event_ms = 0u;
    s_state.challenge.event_seq = 0u;
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

void app_state_set_main_state(app_main_state_t state)
{
    taskENTER_CRITICAL();
    s_state.main_state = state;
    taskEXIT_CRITICAL();
}

app_main_state_t app_state_get_main_state(void)
{
    app_main_state_t state;

    taskENTER_CRITICAL();
    state = s_state.main_state;
    taskEXIT_CRITICAL();
    return state;
}

void app_state_set_challenge(const app_challenge_info_t *challenge)
{
    if (challenge == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    s_state.challenge = *challenge;
    taskEXIT_CRITICAL();
}

void app_state_get_challenge(app_challenge_info_t *challenge)
{
    if (challenge == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    *challenge = s_state.challenge;
    taskEXIT_CRITICAL();
}

void app_state_emit_event(app_event_id_t event_id)
{
    uint32_t event_ms;

    if (event_id == APP_EVENT_NONE) {
        return;
    }

    event_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

    taskENTER_CRITICAL();
    s_state.challenge.last_event = event_id;
    s_state.challenge.last_event_ms = event_ms;
    s_state.challenge.event_seq += 1u;
    taskEXIT_CRITICAL();

    signal_emit(event_id);
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
    case APP_MODE_LINE_TEST:
        return "LINE";
    case APP_MODE_STRAIGHT_TEST:
        return "STRAIGHT";
    case APP_MODE_MAIN:
        return "MAIN";
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

const char *app_main_state_name(app_main_state_t state)
{
    switch (state) {
    case APP_MAIN_STATE_ALIGN:
        return "ALIGN";
    case APP_MAIN_STATE_GAP:
        return "GAP";
    case APP_MAIN_STATE_ARC_TRACK:
        return "ARC";
    case APP_MAIN_STATE_ARC_LOST_LEFT:
        return "ARC_L";
    case APP_MAIN_STATE_ARC_LOST_RIGHT:
        return "ARC_R";
    case APP_MAIN_STATE_ARC_EXIT_ALIGN:
        return "ARC_ALIGN";
    case APP_MAIN_STATE_STOPPED:
        return "STOP";
    case APP_MAIN_STATE_IDLE:
    default:
        return "IDLE";
    }
}

const char *app_challenge_name(app_challenge_t challenge)
{
    switch (challenge) {
    case APP_CHALLENGE_Q1:
        return "Q1";
    case APP_CHALLENGE_Q2:
        return "Q2";
    case APP_CHALLENGE_Q3:
        return "Q3";
    case APP_CHALLENGE_Q4:
        return "Q4";
    case APP_CHALLENGE_NONE:
    default:
        return "--";
    }
}

const char *app_challenge_status_name(app_challenge_status_t status)
{
    switch (status) {
    case APP_CHALLENGE_STATUS_ALIGN:
        return "ALIGN";
    case APP_CHALLENGE_STATUS_RUNNING:
        return "RUN";
    case APP_CHALLENGE_STATUS_DONE:
        return "DONE";
    case APP_CHALLENGE_STATUS_READY:
    default:
        return "READY";
    }
}

const char *app_phase_action_name(app_phase_action_t action)
{
    switch (action) {
    case APP_PHASE_ACTION_ALIGN_START:
        return "ALIGN";
    case APP_PHASE_ACTION_GAP_TRAVERSE:
        return "GAP";
    case APP_PHASE_ACTION_ARC_TRACK:
        return "ARC";
    case APP_PHASE_ACTION_STOP_AND_SIGNAL:
        return "STOP";
    case APP_PHASE_ACTION_NONE:
    default:
        return "--";
    }
}

const char *app_challenge_phase_name(app_challenge_phase_t phase)
{
    switch (phase) {
    case APP_CHALLENGE_PHASE_ALIGN_A_TO_B:
        return "AL_A_B";
    case APP_CHALLENGE_PHASE_ALIGN_A_TO_C:
        return "AL_A_C";
    case APP_CHALLENGE_PHASE_GAP_AB:
        return "GAP_AB";
    case APP_CHALLENGE_PHASE_ARC_BC:
        return "ARC_BC";
    case APP_CHALLENGE_PHASE_GAP_CD:
        return "GAP_CD";
    case APP_CHALLENGE_PHASE_ARC_DA:
        return "ARC_DA";
    case APP_CHALLENGE_PHASE_GAP_AC:
        return "GAP_AC";
    case APP_CHALLENGE_PHASE_ARC_CB:
        return "ARC_CB";
    case APP_CHALLENGE_PHASE_GAP_BD:
        return "GAP_BD";
    case APP_CHALLENGE_PHASE_STOP_A:
        return "STOP_A";
    case APP_CHALLENGE_PHASE_STOP_B:
        return "STOP_B";
    case APP_CHALLENGE_PHASE_IDLE:
    default:
        return "IDLE";
    }
}

const char *app_event_name(app_event_id_t event_id)
{
    switch (event_id) {
    case APP_EVENT_START:
        return "START";
    case APP_EVENT_PASS_A:
        return "PASS_A";
    case APP_EVENT_PASS_B:
        return "PASS_B";
    case APP_EVENT_PASS_C:
        return "PASS_C";
    case APP_EVENT_PASS_D:
        return "PASS_D";
    case APP_EVENT_STOP:
        return "STOP";
    case APP_EVENT_NONE:
    default:
        return "--";
    }
}
