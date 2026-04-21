#include "sensor_task.h"

#include "FreeRTOS.h"
#include "task.h"

#include "app_state.h"
#include "chassis_system.h"
#include "imu_drv.h"
#include "line_sensor.h"
#include "mpu6050.h"
#include "uart_printf.h"

#define SENSOR_TASK_PERIOD_MS  10U

void sensor_task(void *arg)
{
    TickType_t next;
    TickType_t last_retry_tick = 0;
    TickType_t imu_init_tick = 0;
    TickType_t imu_stable_tick = 0;
    uint32_t retry_count = 0u;
    uint8_t imu_initialized = 0u;
    uint8_t imu_stable_latched = 0u;
    imu_t *imu;
    line_sensor_t *line_sensor;
    encoder_t *left_encoder;
    encoder_t *right_encoder;

    (void)arg;

    chassis_system_init();
    imu = chassis_system_get_imu();
    line_sensor = chassis_system_get_line_sensor();
    left_encoder = chassis_system_get_left_encoder();
    right_encoder = chassis_system_get_right_encoder();

    next = xTaskGetTickCount();

    for (;;) {
        chassis_feedback_t feedback;

        if (imu_initialized == 0u) {
            TickType_t now = xTaskGetTickCount();

            if ((retry_count == 0u) ||
                ((now - last_retry_tick) >= pdMS_TO_TICKS(1000))) {
                if (Imu_Init(imu) == 0) {
                    imu_initialized = 1u;
                    imu->ready = 1u;
                    retry_count = 0u;
                    imu_init_tick = now;
                    imu_stable_tick = 0;
                    imu_stable_latched = 0u;
                    uart_printf("sensor: imu init ok\r\n");
                } else {
                    ++retry_count;
                    imu->ready = 0u;
                    uart_printf("sensor: imu init failed, retry=%u\r\n",
                                (unsigned)retry_count);
                }
                last_retry_tick = now;
            }
        } else {
            TickType_t now = xTaskGetTickCount();

            Imu_Refresh(imu);
            if ((imu_stable_latched == 0u) &&
                (imu->ready != 0u) &&
                (imu->gyro_z > -3.0f) && (imu->gyro_z < 3.0f)) {
                imu_stable_tick = now;
                imu_stable_latched = 1u;
            }
            if (MPU6050_IsReady() == 0) {
                imu_initialized = 0u;
                imu->ready = 0u;
                imu_stable_tick = 0;
                imu_stable_latched = 0u;
            }
        }

        LineSensor_Refresh(line_sensor);

        feedback.dt_s = (float)SENSOR_TASK_PERIOD_MS / 1000.0f;
        feedback.left_speed_mps = Encoder_GetSpeedMps(left_encoder);
        feedback.right_speed_mps = Encoder_GetSpeedMps(right_encoder);
        feedback.left_count = Encoder_GetCount(left_encoder);
        feedback.right_count = Encoder_GetCount(right_encoder);
        feedback.imu_ready = imu->ready;
        feedback.yaw_deg = imu->yaw_deg;
        feedback.gyro_z = imu->gyro_z;
        feedback.pitch_deg = imu->pitch_deg;
        feedback.roll_deg = imu->roll_deg;
        feedback.accel_x = imu->accel_x;
        feedback.accel_y = imu->accel_y;
        feedback.accel_z = imu->accel_z;
        feedback.gyro_x = imu->gyro_x;
        feedback.gyro_y = imu->gyro_y;
        feedback.imu_uptime_ms = (imu_initialized != 0u)
            ? (uint32_t)((xTaskGetTickCount() - imu_init_tick) * portTICK_PERIOD_MS)
            : 0u;
        feedback.imu_stable_ms = (imu_stable_latched != 0u)
            ? (uint32_t)((imu_stable_tick - imu_init_tick) * portTICK_PERIOD_MS)
            : 0u;
        feedback.line_bits = line_sensor->bits;
        feedback.line_detected = line_sensor->detected;
        app_state_set_feedback(&feedback);

        vTaskDelayUntil(&next, pdMS_TO_TICKS(SENSOR_TASK_PERIOD_MS));
    }
}
