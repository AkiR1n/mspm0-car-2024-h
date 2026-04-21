#ifndef IMU_DRV_H_
#define IMU_DRV_H_

#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint8_t ready;
    float   yaw_deg;
    float   pitch_deg;
    float   roll_deg;
    float   gyro_x;
    float   gyro_y;
    float   gyro_z;
    float   accel_x;
    float   accel_y;
    float   accel_z;
} imu_t;

int Imu_Init(imu_t *imu);
void Imu_Refresh(imu_t *imu);

#endif
