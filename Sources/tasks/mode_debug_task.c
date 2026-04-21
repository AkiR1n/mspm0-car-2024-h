#include "mode_debug_task.h"

#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app_state.h"
#include "oled_hardware_i2c.h"
#include "uart_printf.h"

#define MODE_DEBUG_TASK_PERIOD_MS   50U
#define MODE_DEBUG_LOG_PERIOD_MS   500U

#define MODE_DEBUG_DEFAULT_MODE     APP_MODE_STOP
#define MODE_DEBUG_DEFAULT_V_MPS    0.00f
#define MODE_DEBUG_DEFAULT_W_RADPS  0.00f

void mode_debug_task(void *arg)
{
    TickType_t next;
    TickType_t last_log_tick = 0;

    (void)arg;

    OLED_Init();
    OLED_Clear();
    app_state_set_mode(MODE_DEBUG_DEFAULT_MODE);
    next = xTaskGetTickCount();

    for (;;) {
        app_state_snapshot_t snapshot;
        chassis_command_t command;
        char buf[24];
        TickType_t now;

        command.stop = (MODE_DEBUG_DEFAULT_MODE == APP_MODE_STOP) ? 1u : 0u;
        command.enable_closed_loop = 1u;
        command.v_mps = MODE_DEBUG_DEFAULT_V_MPS;
        command.w_radps = MODE_DEBUG_DEFAULT_W_RADPS;
        app_state_set_mode(MODE_DEBUG_DEFAULT_MODE);
        app_state_set_command(&command);
        app_state_get_snapshot(&snapshot);

        snprintf(buf, sizeof buf, "M %-6s", app_mode_name(snapshot.mode));
        OLED_ShowString(0, 0, (uint8_t *)buf, 16);

        snprintf(buf, sizeof buf, "v%+5.2f w%+4.1f",
                 (double)snapshot.command.v_mps,
                 (double)snapshot.command.w_radps);
        OLED_ShowString(0, 2, (uint8_t *)buf, 16);

        snprintf(buf, sizeof buf, "L%+4.2f R%+4.2f",
                 (double)snapshot.debug.left_measured_mps,
                 (double)snapshot.debug.right_measured_mps);
        OLED_ShowString(0, 4, (uint8_t *)buf, 16);

        snprintf(buf, sizeof buf, "I%u Y%+5.1f",
                 (unsigned)snapshot.feedback.imu_ready,
                 (double)snapshot.feedback.yaw_deg);
        OLED_ShowString(0, 6, (uint8_t *)buf, 16);

        now = xTaskGetTickCount();
        if ((now - last_log_tick) >= pdMS_TO_TICKS(MODE_DEBUG_LOG_PERIOD_MS)) {
            uart_printf(
                "mode=%s imu=%u/%u ypr=(%.2f,%.2f,%.2f) gz=%.1f up=%lus stable=%lus\r\n",
                app_mode_name(snapshot.mode),
                (unsigned)snapshot.feedback.imu_ready,
                (unsigned)snapshot.feedback.imu_stable,
                (double)snapshot.feedback.yaw_deg,
                (double)snapshot.feedback.pitch_deg,
                (double)snapshot.feedback.roll_deg,
                (double)snapshot.feedback.gyro_z,
                (unsigned long)(snapshot.feedback.imu_uptime_ms / 1000u),
                (unsigned long)(snapshot.feedback.imu_stable_ms / 1000u));
            last_log_tick = now;
        }

        vTaskDelayUntil(&next, pdMS_TO_TICKS(MODE_DEBUG_TASK_PERIOD_MS));
    }
}
