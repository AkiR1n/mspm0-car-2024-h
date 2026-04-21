#include "imu_drv.h"

#include "mpu6050.h"

int Imu_Init(imu_t *imu)
{
    int result;

    if (imu == NULL) {
        return -1;
    }

    result = MPU6050_Init();
    imu->ready = (result == 0) ? 1u : 0u;
    imu->yaw_deg = 0.0f;
    imu->pitch_deg = 0.0f;
    imu->roll_deg = 0.0f;
    imu->gyro_x = 0.0f;
    imu->gyro_y = 0.0f;
    imu->gyro_z = 0.0f;
    imu->accel_x = 0.0f;
    imu->accel_y = 0.0f;
    imu->accel_z = 0.0f;
    return result;
}

void Imu_Refresh(imu_t *imu)
{
    if (imu == NULL) {
        return;
    }

    if (Read_Quad() == 0) {
        imu->ready = (uint8_t)MPU6050_IsReady();
        imu->yaw_deg = yaw;
        imu->pitch_deg = pitch;
        imu->roll_deg = roll;
        imu->gyro_x = (float)gyro[0];
        imu->gyro_y = (float)gyro[1];
        imu->gyro_z = (float)gyro[2];
        imu->accel_x = (float)accel[0];
        imu->accel_y = (float)accel[1];
        imu->accel_z = (float)accel[2];
    }
}
