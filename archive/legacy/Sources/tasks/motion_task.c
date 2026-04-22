#include "motion_task.h"

#include "FreeRTOS.h"
#include "task.h"
#include "ti_msp_dl_config.h"

#include "app_state.h"
#include "encoder.h"
#include "linetracker.h"
#include "motion_control.h"
#include "motor.h"
#include "turn_detection.h"

#define MOTION_TASK_PERIOD_MS  10U
#define MOTION_BASE_SPEED_PPS  900.0f

static void publish_motion_state(const linetracker_state_t *line_state,
                                 motion_mode_t mode)
{
    g_app_state.line_bits = line_state->sensor_bits;
    g_app_state.line_position = line_state->position;
    g_app_state.left_pps = encoder_pps(ENCODER_LEFT);
    g_app_state.right_pps = encoder_pps(ENCODER_RIGHT);
    g_app_state.mode = mode;
    g_app_state.base_speed_pps = MOTION_BASE_SPEED_PPS;
}

void motion_task(void *arg)
{
    (void)arg;

    motor_init(NULL);
    encoder_init(NULL);
    linetracker_init(NULL);
    turn_detection_init(NULL);
    motion_init(NULL);
    motion_set_base_speed_pps(MOTION_BASE_SPEED_PPS);
    g_app_state.base_speed_pps = MOTION_BASE_SPEED_PPS;

    NVIC_EnableIRQ(TIMER_CALC_INST_INT_IRQN);
    DL_TimerA_startCounter(TIMER_CALC_INST);

    TickType_t next = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(MOTION_TASK_PERIOD_MS);
    uint8_t line_was_detected = 0u;

    for (;;) {
        const linetracker_state_t *line_state = linetracker_read();
        const uint32_t now_ms =
            (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        motion_mode_t mode = MOTION_MODE_STOP;
        turn_dir_t turn = TURN_DIR_NONE;

        turn_detection_update(now_ms);
        turn = turn_detection_consume();
        if (turn != TURN_DIR_NONE) {
            g_app_state.last_turn = turn;
        }

        motion_set_base_speed_pps(MOTION_BASE_SPEED_PPS);
        motion_set_yaw_feedback(g_app_state.yaw_deg);

        if (line_state->line_detected) {
            mode = MOTION_MODE_LINE_FOLLOW;
            line_was_detected = 1u;
        } else if (g_app_state.imu_ready) {
            if (line_was_detected) {
                motion_set_target_yaw(g_app_state.yaw_deg);
            }
            mode = MOTION_MODE_YAW_HOLD;
            line_was_detected = 0u;
        } else {
            mode = MOTION_MODE_STOP;
            line_was_detected = 0u;
        }

        motion_set_mode(mode);
        motion_step((float)MOTION_TASK_PERIOD_MS / 1000.0f);
        publish_motion_state(line_state, mode);

        vTaskDelayUntil(&next, period);
    }
}
