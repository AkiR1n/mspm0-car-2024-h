#include "log_task.h"

#include "FreeRTOS.h"
#include "task.h"

#include "app_state.h"
#include "uart_printf.h"

/*
 * log_task — 1 s 打印运行状态，方便串口看板子是否活着。
 */
void log_task(void *arg)
{
    (void)arg;
    for (;;) {
        uart_printf("stat heap=%u imu=%u ypr=(%.1f,%.1f,%.1f) line=0x%02X/%d pps=(%ld,%ld) mode=%s base=%.0f turn=%s\r\n",
                    (unsigned)xPortGetFreeHeapSize(),
                    (unsigned)g_app_state.imu_ready,
                    (double)g_app_state.yaw_deg,
                    (double)g_app_state.pitch_deg,
                    (double)g_app_state.roll_deg,
                    (unsigned)g_app_state.line_bits,
                    (int)g_app_state.line_position,
                    (long)g_app_state.left_pps,
                    (long)g_app_state.right_pps,
                    app_motion_mode_name(g_app_state.mode),
                    (double)g_app_state.base_speed_pps,
                    app_turn_name(g_app_state.last_turn));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
