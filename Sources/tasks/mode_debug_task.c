#include "mode_debug_task.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app_state.h"
#include "encoder_hal.h"
#include "uart_rx.h"
#include "uart_printf.h"

#define MODE_DEBUG_TASK_PERIOD_MS  20U
#define MODE_DEBUG_LOG_PERIOD_MS   100U

static float clampf(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static void mode_debug_apply_command(float left_cmd, float right_cmd)
{
    chassis_command_t command;

    command.stop = 0u;
    command.enable_closed_loop = 0u;
    command.v_mps = 0.0f;
    command.w_radps = 0.0f;

    /* 兼容两种输入：
     * 1. (-100,100) 百分比
     * 2. (-1.0,1.0) 直接 duty
     */
    if ((left_cmd > 1.0f) || (left_cmd < -1.0f) ||
        (right_cmd > 1.0f) || (right_cmd < -1.0f)) {
        left_cmd *= 0.01f;
        right_cmd *= 0.01f;
    }

    command.left_duty = clampf(left_cmd, -1.0f, 1.0f);
    command.right_duty = clampf(right_cmd, -1.0f, 1.0f);
    app_state_set_mode(APP_MODE_WHEEL_TEST);
    app_state_set_command(&command);
}

static int mode_debug_parse_number(const char *text, float *value)
{
    char *endptr;

    if ((text == NULL) || (value == NULL) || (*text == '\0')) {
        return 0;
    }

    *value = strtof(text, &endptr);
    return (endptr != text) && (*endptr == '\0');
}

static void mode_debug_poll_uart(void)
{
    static char left_buf[24];
    static char right_buf[24];
    static uint32_t left_len = 0u;
    static uint32_t right_len = 0u;
    static uint8_t seen_comma = 0u;
    char ch;

    while (uart_rx_get_char(&ch) != 0) {
        if (ch == '\r' || ch == '\n') {
            float left_cmd;
            float right_cmd;

            if ((left_len == 0u) && (right_len == 0u) && (seen_comma == 0u)) {
                continue;
            }

            left_buf[left_len] = '\0';
            right_buf[right_len] = '\0';

            if ((seen_comma != 0u) &&
                mode_debug_parse_number(left_buf, &left_cmd) != 0 &&
                mode_debug_parse_number(right_buf, &right_cmd) != 0) {
                mode_debug_apply_command(left_cmd, right_cmd);
                uart_printf("cmd=(%.3f,%.3f)\r\n", (double)left_cmd, (double)right_cmd);
            } else {
                uart_printf("cmd parse error: L='%s' R='%s'\r\n", left_buf, right_buf);
            }
            left_len = 0u;
            right_len = 0u;
            seen_comma = 0u;
            continue;
        }

        if (ch == ',') {
            seen_comma = 1u;
            continue;
        }

        if ((ch == ' ') || (ch == '\t')) {
            continue;
        }

        if (((ch >= '0') && (ch <= '9')) || (ch == '.') || (ch == '-') || (ch == '+')) {
            if (seen_comma == 0u) {
                if (left_len < (sizeof(left_buf) - 1u)) {
                    left_buf[left_len++] = ch;
                }
            } else if (right_len < (sizeof(right_buf) - 1u)) {
                right_buf[right_len++] = ch;
            }
            continue;
        }

        left_len = 0u;
        right_len = 0u;
        seen_comma = 0u;
    }
}

void mode_debug_task(void *arg)
{
    TickType_t next;
    TickType_t last_log_tick = 0;
    uint32_t last_irq_count = 0u;

    (void)arg;

    app_state_set_mode(APP_MODE_WHEEL_TEST);
    mode_debug_apply_command(0.0f, 0.0f);
    uart_rx_init();
    uart_printf("wheel test ready, send left_duty,right_duty\r\n");
    uart_printf("csv_header=ms,left_cmd,right_cmd,left_duty,right_duty,left_mps,right_mps,left_count,right_count,irq_per_s,sample_tick\r\n");
    next = xTaskGetTickCount();

    for (;;) {
        app_state_snapshot_t snapshot;
        TickType_t now;

        mode_debug_poll_uart();
        app_state_get_snapshot(&snapshot);

        now = xTaskGetTickCount();
        if ((now - last_log_tick) >= pdMS_TO_TICKS(MODE_DEBUG_LOG_PERIOD_MS)) {
            uint32_t irq_count = EncoderHal_GetGpioIrqCount();
            uint32_t irq_per_sec =
                (irq_count - last_irq_count) * (1000u / MODE_DEBUG_LOG_PERIOD_MS);
            uint32_t uptime_ms = (uint32_t)now * (uint32_t)portTICK_PERIOD_MS;

            uart_printf(
                "csv,%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%ld,%ld,%lu,%lu\r\n",
                (unsigned long)uptime_ms,
                (double)snapshot.command.left_duty,
                (double)snapshot.command.right_duty,
                (double)snapshot.debug.left_motor_duty,
                (double)snapshot.debug.right_motor_duty,
                (double)snapshot.feedback.left_speed_mps,
                (double)snapshot.feedback.right_speed_mps,
                (long)snapshot.feedback.left_count,
                (long)snapshot.feedback.right_count,
                (unsigned long)irq_per_sec,
                (unsigned long)EncoderHal_GetSampleTickCount());
            last_irq_count = irq_count;
            last_log_tick = now;
        }

        vTaskDelayUntil(&next, pdMS_TO_TICKS(MODE_DEBUG_TASK_PERIOD_MS));
    }
}
