#include "imu_task.h"

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app_state.h"
#include "mpu6050.h"
#include "uart_printf.h"

#define IMU_TASK_PERIOD_MS  10U

void imu_task(void *arg)
{
    (void)arg;

    uint32_t retry_count = 0u;
    while (MPU6050_Init() != 0) {
        ++retry_count;
        g_app_state.imu_ready = 0u;
        g_app_state.yaw_deg = 0.0f;
        g_app_state.pitch_deg = 0.0f;
        g_app_state.roll_deg = 0.0f;
        uart_printf("imu: init failed, retry=%u\r\n", (unsigned)retry_count);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    g_app_state.imu_ready = 1u;
    uart_printf("imu: init ok\r\n");

    TickType_t next = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(IMU_TASK_PERIOD_MS);

    for (;;) {
        if (Read_Quad() == 0) {
            g_app_state.imu_ready = (uint8_t)MPU6050_IsReady();
            g_app_state.yaw_deg = yaw;
            g_app_state.pitch_deg = pitch;
            g_app_state.roll_deg = roll;
        }

        vTaskDelayUntil(&next, period);
    }
}
