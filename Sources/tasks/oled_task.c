#include "oled_task.h"

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app_state.h"
#include "oled_hardware_i2c.h"

#define OLED_TASK_PERIOD_MS 100U
#define OLED_TEXT_COLS      16U
#define OLED_FORCE_REFRESH_TICKS \
    (1000U / OLED_TASK_PERIOD_MS)

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

static const char *display_task_name(const app_state_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return "NONE";
    }
    if (snapshot->challenge.active != APP_CHALLENGE_NONE) {
        return app_challenge_name(snapshot->challenge.active);
    }
    if (snapshot->challenge.selected != APP_CHALLENGE_NONE) {
        return app_challenge_name(snapshot->challenge.selected);
    }
    if (snapshot->mode == APP_MODE_STRAIGHT_TEST) {
        return "STRAIGHT";
    }
    if (snapshot->mode == APP_MODE_LINE_TEST) {
        return "LINE";
    }
    if (snapshot->mode == APP_MODE_WHEEL_TEST) {
        return "WHEEL";
    }
    if (snapshot->mode == APP_MODE_WHEEL_SPEED_TEST) {
        return "SPEED";
    }
    return "NONE";
}

static const char *display_status_name(const app_state_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return "UNKNOWN";
    }
    if ((snapshot->challenge.active != APP_CHALLENGE_NONE) ||
        (snapshot->challenge.selected != APP_CHALLENGE_NONE)) {
        return app_challenge_status_name(snapshot->challenge.status);
    }
    return app_mode_name(snapshot->mode);
}

static const char *display_imu_status(const chassis_feedback_t *feedback)
{
    if (feedback == NULL) {
        return "ERR";
    }
    if ((feedback->imu_ready != 0u) && (feedback->imu_stable != 0u)) {
        return "OK";
    }
    if (feedback->imu_ready != 0u) {
        return "WAIT";
    }
    return "ERR";
}

void oled_task(void *arg)
{
    TickType_t next;
    app_state_snapshot_t snapshot;
    char line0[OLED_TEXT_COLS + 1u];
    char line1[OLED_TEXT_COLS + 1u];
    char line2[OLED_TEXT_COLS + 1u];
    char line3[OLED_TEXT_COLS + 1u];
    char prev0[OLED_TEXT_COLS + 1u] = {0};
    char prev1[OLED_TEXT_COLS + 1u] = {0};
    char prev2[OLED_TEXT_COLS + 1u] = {0};
    char prev3[OLED_TEXT_COLS + 1u] = {0};
    uint8_t refresh_ticks = OLED_FORCE_REFRESH_TICKS;

    (void)arg;

    OLED_Init();
    OLED_Clear();

    next = xTaskGetTickCount();

    for (;;) {
        app_state_get_snapshot(&snapshot);

        snprintf(line0,
                 sizeof(line0),
                 "TASK:%s",
                 display_task_name(&snapshot));
        snprintf(line1,
                 sizeof(line1),
                 "STATE:%s",
                 display_status_name(&snapshot));
        snprintf(line2,
                 sizeof(line2),
                 "IMU:%s",
                 display_imu_status(&snapshot.feedback));
        snprintf(line3,
                 sizeof(line3),
                 "YAW:%+7.1f",
                 (double)snapshot.feedback.yaw_deg);

        pad_text_line(line0, sizeof(line0));
        pad_text_line(line1, sizeof(line1));
        pad_text_line(line2, sizeof(line2));
        pad_text_line(line3, sizeof(line3));

        refresh_ticks++;
        if (refresh_ticks >= OLED_FORCE_REFRESH_TICKS) {
            refresh_ticks = 0u;
        }

        if ((refresh_ticks == 0u) || (strcmp(prev0, line0) != 0)) {
            OLED_ShowString16Line(0, line0, OLED_TEXT_COLS);
            memcpy(prev0, line0, sizeof(prev0));
        }
        if ((refresh_ticks == 0u) || (strcmp(prev1, line1) != 0)) {
            OLED_ShowString16Line(2, line1, OLED_TEXT_COLS);
            memcpy(prev1, line1, sizeof(prev1));
        }
        if ((refresh_ticks == 0u) || (strcmp(prev2, line2) != 0)) {
            OLED_ShowString16Line(4, line2, OLED_TEXT_COLS);
            memcpy(prev2, line2, sizeof(prev2));
        }
        if ((refresh_ticks == 0u) || (strcmp(prev3, line3) != 0)) {
            OLED_ShowString16Line(6, line3, OLED_TEXT_COLS);
            memcpy(prev3, line3, sizeof(prev3));
        }

        vTaskDelayUntil(&next, pdMS_TO_TICKS(OLED_TASK_PERIOD_MS));
    }
}
