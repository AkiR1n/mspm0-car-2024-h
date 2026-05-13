#ifndef IMU_DRV_H_
#define IMU_DRV_H_

#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint8_t auto_calibration;
    uint16_t warmup_ms;
    float stable_gyro_threshold;
    uint16_t stable_hold_ms;
    uint8_t estimate_gyro_bias;
    uint8_t apply_dmp_bias;
    uint8_t zero_yaw_on_stable;
    float   gyro_z_sign;
    float   gyro_sens_override;
} imu_cfg_t;

typedef struct {
    uint8_t ready;
    uint8_t stable;
    float   yaw_deg;
    float   pitch_deg;
    float   roll_deg;
    float   gyro_x;
    float   gyro_y;
    float   gyro_z;
    float   accel_x;
    float   accel_y;
    float   accel_z;
    uint32_t uptime_ms;
    uint32_t stable_ms;
    uint32_t stable_hold_accum_ms;
    uint8_t bias_committed;
    uint8_t yaw_zeroed;
    uint32_t gyro_bias_sample_count;
    float yaw_deg_raw;
    float yaw_zero_deg;
    float yaw_rel_deg;
    float gyro_sens_lsb_per_dps;
    float gyro_x_bias;
    float gyro_y_bias;
    float gyro_z_bias;
    float gyro_x_raw;
    float gyro_y_raw;
    float gyro_z_raw;
    float gyro_x_bias_accum;
    float gyro_y_bias_accum;
    float gyro_z_bias_accum;
    imu_cfg_t cfg;
} imu_t;

extern const imu_cfg_t IMU_CONFIG_DEFAULT;

int Imu_Init(imu_t *imu, const imu_cfg_t *cfg);
void Imu_Refresh(imu_t *imu, uint32_t dt_ms);
void Imu_SetGyroZSign(imu_t *imu, float sign);
void Imu_SetGyroSensOverride(imu_t *imu, float sens);

#endif
