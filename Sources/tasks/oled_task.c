#include "oled_task.h"

#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app_state.h"
#include "oled_hardware_i2c.h"

/*
 * oled_task — 100 ms 刷新
 *   line 0: IMU ready
 *   line 2: yaw
 *   line 4: line bits + position
 *   line 6: motion mode + base speed
 */
void oled_task(void *arg)
{
    (void)arg;
    OLED_Init();
    OLED_Clear();

    TickType_t next = xTaskGetTickCount();
    char buf[24];

    for (;;) {
        snprintf(buf, sizeof buf, "IMU %-10s",
                 g_app_state.imu_ready ? "READY" : "RETRY");
        OLED_ShowString(0, 0, (uint8_t *)buf, 16);

        snprintf(buf, sizeof buf, "Y %+7.1f   ", (double)g_app_state.yaw_deg);
        OLED_ShowString(0, 2, (uint8_t *)buf, 16);

        snprintf(buf, sizeof buf, "L%02X P%+4d  ",
                 (unsigned)g_app_state.line_bits,
                 (int)g_app_state.line_position);
        OLED_ShowString(0, 4, (uint8_t *)buf, 16);

        snprintf(buf, sizeof buf, "M%-4s %4.0f ",
                 app_motion_mode_name(g_app_state.mode),
                 (double)g_app_state.base_speed_pps);
        OLED_ShowString(0, 6, (uint8_t *)buf, 16);

        vTaskDelayUntil(&next, pdMS_TO_TICKS(100));
    }
}
