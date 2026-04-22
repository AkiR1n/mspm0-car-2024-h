#include "oled_task.h"

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app_state.h"
#include "oled_hardware_i2c.h"

#define OLED_TASK_PERIOD_MS 100U
#define OLED_TEXT_COLS      21U

static void pad_text_line(char *text, size_t size)
{
    size_t len;
    size_t i;

    if ((text == NULL) || (size == 0u)) {
        return;
    }

    len = strlen(text);
    if (len >= size) {
        text[size - 1u] = '\0';
        return;
    }

    for (i = len; i + 1u < size; ++i) {
        text[i] = ' ';
    }
    text[size - 1u] = '\0';
}

static int round_to_int(float value)
{
    if (value >= 0.0f) {
        return (int)(value + 0.5f);
    }
    return (int)(value - 0.5f);
}

void oled_task(void *arg)
{
    TickType_t next;
    app_state_snapshot_t snapshot;
    char line0[OLED_TEXT_COLS + 1u];
    char line1[OLED_TEXT_COLS + 1u];
    char line2[OLED_TEXT_COLS + 1u];
    char line3[OLED_TEXT_COLS + 1u];

    (void)arg;

    OLED_Init();
    OLED_Clear();

    next = xTaskGetTickCount();

    for (;;) {
        app_state_get_snapshot(&snapshot);

        snprintf(line0,
                 sizeof(line0),
                 "Q:%s %s L:%u/%u",
                 app_challenge_name(snapshot.challenge.selected),
                 app_challenge_status_name(snapshot.challenge.status),
                 (unsigned)snapshot.challenge.lap_index,
                 (unsigned)snapshot.challenge.lap_total);
        snprintf(line1,
                 sizeof(line1),
                 "P:%s A:%s",
                 app_challenge_phase_name(snapshot.challenge.phase),
                 app_phase_action_name(snapshot.challenge.action));
        snprintf(line2,
                 sizeof(line2),
                 "E:%s T:%lu",
                 app_event_name(snapshot.challenge.last_event),
                 (unsigned long)snapshot.challenge.last_event_ms);

        if (snapshot.challenge.status == APP_CHALLENGE_STATUS_READY) {
            snprintf(line3,
                     sizeof(line3),
                     "READY IMU:%u ST:%u",
                     (unsigned)snapshot.feedback.imu_ready,
                     (unsigned)snapshot.feedback.imu_stable);
        } else if (snapshot.challenge.action == APP_PHASE_ACTION_GAP_TRAVERSE) {
            snprintf(line3,
                     sizeof(line3),
                     "H:%+4d D:%1.2f",
                     round_to_int(snapshot.challenge.heading_error_deg),
                     (double)snapshot.challenge.phase_distance_m);
        } else if (snapshot.challenge.action == APP_PHASE_ACTION_ARC_TRACK) {
            snprintf(line3,
                     sizeof(line3),
                     "L:%+3d D:%1.2f",
                     (int)snapshot.feedback.line_position,
                     (double)snapshot.challenge.phase_distance_m);
        } else if (snapshot.challenge.action == APP_PHASE_ACTION_ALIGN_START) {
            snprintf(line3,
                     sizeof(line3),
                     "ALIGN Y:%+4d ST:%u",
                     round_to_int(snapshot.feedback.yaw_deg),
                     (unsigned)snapshot.feedback.imu_stable);
        } else {
            snprintf(line3,
                     sizeof(line3),
                     "M:%s S:%s",
                     app_mode_name(snapshot.mode),
                     app_main_state_name(snapshot.main_state));
        }

        pad_text_line(line0, sizeof(line0));
        pad_text_line(line1, sizeof(line1));
        pad_text_line(line2, sizeof(line2));
        pad_text_line(line3, sizeof(line3));

        OLED_ShowString(0, 0, (uint8_t *)line0, 8);
        OLED_ShowString(0, 1, (uint8_t *)line1, 8);
        OLED_ShowString(0, 2, (uint8_t *)line2, 8);
        OLED_ShowString(0, 3, (uint8_t *)line3, 8);

        vTaskDelayUntil(&next, pdMS_TO_TICKS(OLED_TASK_PERIOD_MS));
    }
}
