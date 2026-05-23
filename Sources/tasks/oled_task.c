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

static void format_challenge_slot(char slot[5],
                                  app_challenge_t challenge,
                                  const app_challenge_info_t *info)
{
    uint8_t running;

    if ((slot == NULL) || (info == NULL)) {
        return;
    }

    running = ((info->status == APP_CHALLENGE_STATUS_ALIGN) ||
               (info->status == APP_CHALLENGE_STATUS_RUNNING))
                  ? 1u
                  : 0u;

    if ((running != 0u) && (info->active == challenge)) {
        snprintf(slot, 5u, "*%s*", app_challenge_name(challenge));
    } else if (info->selected == challenge) {
        snprintf(slot, 5u, "[%s]", app_challenge_name(challenge));
    } else {
        snprintf(slot, 5u, " %s ", app_challenge_name(challenge));
    }
}

static void format_challenge_line(char *line,
                                  size_t size,
                                  const app_challenge_info_t *info)
{
    char q1[5];
    char q2[5];
    char q3[5];
    char q4[5];

    if ((line == NULL) || (size == 0u) || (info == NULL)) {
        return;
    }

    format_challenge_slot(q1, APP_CHALLENGE_Q1, info);
    format_challenge_slot(q2, APP_CHALLENGE_Q2, info);
    format_challenge_slot(q3, APP_CHALLENGE_Q3, info);
    format_challenge_slot(q4, APP_CHALLENGE_Q4, info);

    snprintf(line, size, "%s %s %s %s", q1, q2, q3, q4);
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

        format_challenge_line(line0, sizeof(line0), &snapshot.challenge);
        snprintf(line1,
                 sizeof(line1),
                 "%s L:%u/%u CP:%u",
                 app_challenge_status_name(snapshot.challenge.status),
                 (unsigned)snapshot.challenge.lap_index,
                 (unsigned)snapshot.challenge.lap_total,
                 (unsigned)snapshot.challenge.checkpoint_count);
        snprintf(line2,
                 sizeof(line2),
                 "%s D:%1.2f",
                 app_challenge_phase_name(snapshot.challenge.phase),
                 (double)snapshot.challenge.phase_distance_m);

        if (snapshot.challenge.status == APP_CHALLENGE_STATUS_READY) {
            snprintf(line3,
                     sizeof(line3),
                     "IMU:%u/%u M:%s",
                     (unsigned)snapshot.feedback.imu_ready,
                     (unsigned)snapshot.feedback.imu_stable,
                     app_mode_name(snapshot.mode));
        } else if (snapshot.challenge.action == APP_PHASE_ACTION_GAP_TRAVERSE) {
            snprintf(line3,
                     sizeof(line3),
                     "H:%+4d Y:%+4d",
                     round_to_int(snapshot.challenge.heading_error_deg),
                     round_to_int(snapshot.feedback.yaw_deg));
        } else if (snapshot.challenge.action == APP_PHASE_ACTION_ARC_TRACK) {
            snprintf(line3,
                     sizeof(line3),
                     "LINE:%+3d DET:%u",
                     (int)snapshot.feedback.line_position,
                     (unsigned)snapshot.feedback.line_detected);
        } else if (snapshot.challenge.action == APP_PHASE_ACTION_ALIGN_START) {
            snprintf(line3,
                     sizeof(line3),
                     "Y:%+4d IMU:%u/%u",
                     round_to_int(snapshot.feedback.yaw_deg),
                     (unsigned)snapshot.feedback.imu_ready,
                     (unsigned)snapshot.feedback.imu_stable);
        } else {
            snprintf(line3,
                     sizeof(line3),
                     "EV:%s",
                     app_event_name(snapshot.challenge.last_event));
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
