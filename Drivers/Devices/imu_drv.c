#include "imu_drv.h"

#include "mpu6050.h"

const imu_cfg_t IMU_CONFIG_DEFAULT = {
    .auto_calibration = 0u,
    .warmup_ms = 1200u,
    .stable_gyro_threshold = 2.0f,
    .stable_hold_ms = 600u,
    .estimate_gyro_bias = 1u,
    .apply_dmp_bias = 0u,
    .zero_yaw_on_stable = 1u,
};

static float Imu_WrapAngleDeg(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static void Imu_CommitGyroBias(imu_t *imu)
{
    float gyro_sens;
    long bias_q16[3];

    if ((imu == NULL) || (imu->gyro_bias_sample_count == 0u)) {
        return;
    }

    imu->gyro_x_bias = imu->gyro_x_bias_accum / (float)imu->gyro_bias_sample_count;
    imu->gyro_y_bias = imu->gyro_y_bias_accum / (float)imu->gyro_bias_sample_count;
    imu->gyro_z_bias = imu->gyro_z_bias_accum / (float)imu->gyro_bias_sample_count;

    if ((imu->cfg.apply_dmp_bias == 0u) || (imu->cfg.auto_calibration != 0u)) {
        imu->bias_committed = 1u;
        return;
    }

    if (MPU6050_GetGyroSens(&gyro_sens) != 0) {
        imu->bias_committed = 1u;
        return;
    }

    bias_q16[0] = (long)((imu->gyro_x_bias / gyro_sens) * 65536.0f);
    bias_q16[1] = (long)((imu->gyro_y_bias / gyro_sens) * 65536.0f);
    bias_q16[2] = (long)((imu->gyro_z_bias / gyro_sens) * 65536.0f);
    (void)MPU6050_SetDmpGyroBiasQ16(bias_q16);
    imu->bias_committed = 1u;
}

static void Imu_ResetData(imu_t *imu)
{
    imu->ready = 0u;
    imu->stable = 0u;
    imu->yaw_deg = 0.0f;
    imu->yaw_deg_raw = 0.0f;
    imu->yaw_rel_deg = 0.0f;
    imu->pitch_deg = 0.0f;
    imu->roll_deg = 0.0f;
    imu->gyro_x = 0.0f;
    imu->gyro_y = 0.0f;
    imu->gyro_z = 0.0f;
    imu->accel_x = 0.0f;
    imu->accel_y = 0.0f;
    imu->accel_z = 0.0f;
    imu->uptime_ms = 0u;
    imu->stable_ms = 0u;
    imu->stable_hold_accum_ms = 0u;
    imu->bias_committed = 0u;
    imu->yaw_zeroed = 0u;
    imu->gyro_bias_sample_count = 0u;
    imu->yaw_zero_deg = 0.0f;
    imu->gyro_sens_lsb_per_dps = 131.0f;
    imu->gyro_x_bias = 0.0f;
    imu->gyro_y_bias = 0.0f;
    imu->gyro_z_bias = 0.0f;
    imu->gyro_x_raw = 0.0f;
    imu->gyro_y_raw = 0.0f;
    imu->gyro_z_raw = 0.0f;
    imu->gyro_x_bias_accum = 0.0f;
    imu->gyro_y_bias_accum = 0.0f;
    imu->gyro_z_bias_accum = 0.0f;
}

int Imu_Init(imu_t *imu, const imu_cfg_t *cfg)
{
    int result;
    float gyro_sens;
    const mpu6050_config_t *mpu_cfg;

    if (imu == NULL) {
        return -1;
    }

    imu->cfg = (cfg != NULL) ? *cfg : IMU_CONFIG_DEFAULT;
    Imu_ResetData(imu);

    mpu_cfg = (imu->cfg.auto_calibration != 0u)
        ? &MPU6050_CONFIG_DEFAULT
        : &MPU6050_CONFIG_FAST_START;
    result = MPU6050_InitWithConfig(mpu_cfg);
    imu->ready = (result == 0) ? 1u : 0u;
    if ((result == 0) && (MPU6050_GetGyroSens(&gyro_sens) == 0) &&
        (gyro_sens > 0.0f)) {
        imu->gyro_sens_lsb_per_dps = gyro_sens;
    }
    return result;
}

void Imu_Refresh(imu_t *imu, uint32_t dt_ms)
{
    float corrected_gyro_x_raw;
    float corrected_gyro_y_raw;
    float corrected_gyro_z_raw;
    float abs_gyro_x;
    float abs_gyro_y;
    float abs_gyro_z;
    float dt_s;

    if (imu == NULL) {
        return;
    }

    if (imu->ready == 0u) {
        return;
    }

    if (Read_Quad() == 0) {
        imu->ready = (uint8_t)MPU6050_IsReady();
        imu->yaw_deg_raw = yaw;
        imu->pitch_deg = pitch;
        imu->roll_deg = roll;
        imu->gyro_x_raw = (float)gyro[0];
        imu->gyro_y_raw = (float)gyro[1];
        imu->gyro_z_raw = (float)gyro[2];
        imu->accel_x = (float)accel[0];
        imu->accel_y = (float)accel[1];
        imu->accel_z = (float)accel[2];
        imu->uptime_ms += dt_ms;

        if ((imu->cfg.estimate_gyro_bias != 0u) &&
            (imu->bias_committed == 0u) &&
            (imu->uptime_ms <= imu->cfg.warmup_ms)) {
            imu->gyro_x_bias_accum += imu->gyro_x_raw;
            imu->gyro_y_bias_accum += imu->gyro_y_raw;
            imu->gyro_z_bias_accum += imu->gyro_z_raw;
            imu->gyro_bias_sample_count += 1u;
        }

        if ((imu->cfg.estimate_gyro_bias != 0u) &&
            (imu->gyro_bias_sample_count > 0u) &&
            (imu->uptime_ms >= imu->cfg.warmup_ms) &&
            (imu->bias_committed == 0u)) {
            Imu_CommitGyroBias(imu);
            imu->stable_hold_accum_ms = 0u;
        } else if ((imu->cfg.estimate_gyro_bias == 0u) &&
                   (imu->uptime_ms >= imu->cfg.warmup_ms) &&
                   (imu->bias_committed == 0u)) {
            imu->bias_committed = 1u;
            imu->stable_hold_accum_ms = 0u;
        }

        corrected_gyro_x_raw = imu->gyro_x_raw - imu->gyro_x_bias;
        corrected_gyro_y_raw = imu->gyro_y_raw - imu->gyro_y_bias;
        corrected_gyro_z_raw = imu->gyro_z_raw - imu->gyro_z_bias;

        imu->gyro_x = corrected_gyro_x_raw / imu->gyro_sens_lsb_per_dps;
        imu->gyro_y = corrected_gyro_y_raw / imu->gyro_sens_lsb_per_dps;
        imu->gyro_z = corrected_gyro_z_raw / imu->gyro_sens_lsb_per_dps;

        if (imu->bias_committed != 0u) {
            dt_s = (float)dt_ms / 1000.0f;
            imu->yaw_rel_deg = Imu_WrapAngleDeg(imu->yaw_rel_deg + (imu->gyro_z * dt_s));
            imu->yaw_deg = imu->yaw_rel_deg;
        } else {
            imu->yaw_deg = imu->yaw_deg_raw;
        }

        abs_gyro_x = (imu->gyro_x >= 0.0f) ? imu->gyro_x : -imu->gyro_x;
        abs_gyro_y = (imu->gyro_y >= 0.0f) ? imu->gyro_y : -imu->gyro_y;
        abs_gyro_z = (imu->gyro_z >= 0.0f) ? imu->gyro_z : -imu->gyro_z;

        if ((imu->bias_committed != 0u) &&
            (imu->uptime_ms >= imu->cfg.warmup_ms) &&
            (abs_gyro_x <= imu->cfg.stable_gyro_threshold) &&
            (abs_gyro_y <= imu->cfg.stable_gyro_threshold) &&
            (abs_gyro_z <= imu->cfg.stable_gyro_threshold)) {
            if (imu->stable_hold_accum_ms < imu->cfg.stable_hold_ms) {
                imu->stable_hold_accum_ms += dt_ms;
            }
            if ((imu->stable == 0u) &&
                (imu->stable_hold_accum_ms >= imu->cfg.stable_hold_ms)) {
                imu->stable = 1u;
                imu->stable_ms = imu->uptime_ms;
                if ((imu->cfg.zero_yaw_on_stable != 0u) &&
                    (imu->yaw_zeroed == 0u)) {
                    imu->yaw_zero_deg = imu->yaw_deg_raw;
                    imu->yaw_zeroed = 1u;
                    imu->yaw_rel_deg = 0.0f;
                    imu->yaw_deg = 0.0f;
                }
            }
        } else {
            imu->stable_hold_accum_ms = 0u;
        }
    }
}
