#include "sensor_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "mpu6050.h"
#include "linetracker.h"
#include "uart_printf.h"

/*
 * sensor_task — 10 ms 周期
 *   - 读 7 路循迹（直接写 g_lineTracker）
 *   - 读 MPU6050 DMP（更新全局 yaw/pitch/roll）
 *
 * 关键坑：MPU6050_Init 内部含大量 mspm0_delay_ms → vTaskDelay，初始化
 * 耗时 5–10 s 正常。必须放在任务体内（而不是 main()）以保证调度器已启动。
 */
void sensor_task(void *arg)
{
    (void)arg;
    LineTracker_Init();
    const TickType_t period = pdMS_TO_TICKS(10);
    uint32_t retry_count = 0;

    while (MPU6050_Init() != 0) {
        ++retry_count;
        uart_printf("sensor: MPU6050 init failed, retry=%u\r\n",
                    (unsigned)retry_count);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    uart_printf("sensor: MPU6050 init ok\r\n");

    TickType_t next = xTaskGetTickCount();

    for (;;) {
        LineTracker_ReadSensors();
        (void)Read_Quad();    /* 更新 pitch/roll/yaw 全局 */
        vTaskDelayUntil(&next, period);
    }
}
