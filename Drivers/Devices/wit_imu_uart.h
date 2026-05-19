#ifndef WIT_IMU_UART_H_
#define WIT_IMU_UART_H_

#include <stdint.h>

typedef struct {
    float yaw_deg;
    float pitch_deg;
    float roll_deg;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    uint8_t has_angle;
    uint8_t has_gyro;
    uint32_t sample_seq;
} wit_imu_sample_t;

typedef struct {
    uint32_t rx_bytes;
    uint32_t rx_overflow;
    uint32_t hw_overrun;
    uint32_t sync_drop;
    uint32_t checksum_error;
    uint32_t frame_angle;
    uint32_t frame_gyro;
    uint32_t frame_other;
    uint32_t sample_seq;
} wit_imu_uart_stats_t;

void WitImuUart_Init(void);
void WitImuUart_IrqHandler(void);
uint8_t WitImuUart_ReadLatest(wit_imu_sample_t *sample);
void WitImuUart_GetStats(wit_imu_uart_stats_t *stats);
void WitImuUart_ClearStats(void);

#endif
