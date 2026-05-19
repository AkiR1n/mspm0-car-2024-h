#include "imu_drv.h"

#include "wit_imu_uart.h"

const imu_cfg_t IMU_CONFIG_DEFAULT = {
    .auto_calibration = 0u,
    .warmup_ms = 300u,
    .stable_gyro_threshold = 2.0f,
    .stable_hold_ms = 300u,
    .estimate_gyro_bias = 0u,
    .apply_dmp_bias = 0u,
    .zero_yaw_on_stable = 1u,
    .gyro_z_sign = -1.0f,
    .gyro_sens_override = 0.0f,
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

static float Imu_AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
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
    imu->bias_committed = 1u;
    imu->yaw_zeroed = 0u;
    imu->gyro_bias_sample_count = 0u;
    imu->yaw_zero_deg = 0.0f;
    imu->gyro_sens_lsb_per_dps = 0.0f;
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
    if (imu == NULL) {
        return -1;
    }

    imu->cfg = (cfg != NULL) ? *cfg : IMU_CONFIG_DEFAULT;
    Imu_ResetData(imu);
    WitImuUart_Init();

    /* UART is initialized here; ready becomes true after the first valid frame. */
    imu->ready = 1u;
    return 0;
}

void Imu_Refresh(imu_t *imu, uint32_t dt_ms)
{
    static uint32_t s_last_sample_seq;
    wit_imu_sample_t sample;
    uint8_t have_sample;
    uint8_t fresh_sample;
    float raw_relative_yaw;
    float abs_gyro_z;

    if (imu == NULL) {
        return;
    }

    imu->uptime_ms += dt_ms;

    have_sample = WitImuUart_ReadLatest(&sample);
    fresh_sample = ((have_sample != 0u) && (sample.sample_seq != s_last_sample_seq)) ? 1u : 0u;
    if (fresh_sample == 0u) {
        return;
    }
    s_last_sample_seq = sample.sample_seq;

    imu->ready = ((sample.has_angle != 0u) || (sample.has_gyro != 0u)) ? 1u : 0u;
    if (sample.has_angle != 0u) {
        imu->yaw_deg_raw = sample.yaw_deg;
        imu->pitch_deg = sample.pitch_deg;
        imu->roll_deg = sample.roll_deg;
    }
    if (sample.has_gyro != 0u) {
        imu->gyro_x = sample.gyro_x_dps;
        imu->gyro_y = sample.gyro_y_dps;
        imu->gyro_z = sample.gyro_z_dps * imu->cfg.gyro_z_sign;
        imu->gyro_x_raw = sample.gyro_x_dps;
        imu->gyro_y_raw = sample.gyro_y_dps;
        imu->gyro_z_raw = sample.gyro_z_dps;
    }

    raw_relative_yaw = Imu_WrapAngleDeg(imu->yaw_deg_raw - imu->yaw_zero_deg);
    imu->yaw_rel_deg = raw_relative_yaw * imu->cfg.gyro_z_sign;
    imu->yaw_deg = Imu_WrapAngleDeg(imu->yaw_rel_deg);

    abs_gyro_z = Imu_AbsFloat(imu->gyro_z);
    if ((sample.has_angle != 0u) &&
        (sample.has_gyro != 0u) &&
        (imu->uptime_ms >= imu->cfg.warmup_ms) &&
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
    } else if (imu->stable == 0u) {
        imu->stable_hold_accum_ms = 0u;
    }
}

void Imu_SetGyroZSign(imu_t *imu, float sign)
{
    if (imu != NULL) {
        imu->cfg.gyro_z_sign = (sign >= 0.0f) ? 1.0f : -1.0f;
    }
}

void Imu_SetGyroSensOverride(imu_t *imu, float sens)
{
    if (imu != NULL) {
        imu->cfg.gyro_sens_override = sens;
    }
}
