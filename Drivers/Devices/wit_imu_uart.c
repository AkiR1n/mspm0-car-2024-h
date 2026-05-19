#include "wit_imu_uart.h"

#include <stddef.h>
#include <string.h>

#include "ti_msp_dl_config.h"

#define WIT_RX_BUFFER_SIZE 256u
#define WIT_FRAME_SIZE     11u
#define WIT_FRAME_HEAD     0x55u
#define WIT_FRAME_GYRO     0x52u
#define WIT_FRAME_ANGLE    0x53u

static volatile uint8_t s_rx_buf[WIT_RX_BUFFER_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static volatile wit_imu_uart_stats_t s_stats;
static wit_imu_sample_t s_latest;

static int16_t le_i16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint8_t checksum_ok(const uint8_t frame[WIT_FRAME_SIZE])
{
    uint8_t sum = 0u;
    uint8_t i;

    for (i = 0u; i < (WIT_FRAME_SIZE - 1u); ++i) {
        sum = (uint8_t)(sum + frame[i]);
    }

    return (sum == frame[WIT_FRAME_SIZE - 1u]) ? 1u : 0u;
}

static void parse_frame(const uint8_t frame[WIT_FRAME_SIZE])
{
    if (checksum_ok(frame) == 0u) {
        ++s_stats.checksum_error;
        return;
    }

    switch (frame[1]) {
    case WIT_FRAME_GYRO:
        s_latest.gyro_x_dps = (float)le_i16(&frame[2]) / 32768.0f * 2000.0f;
        s_latest.gyro_y_dps = (float)le_i16(&frame[4]) / 32768.0f * 2000.0f;
        s_latest.gyro_z_dps = (float)le_i16(&frame[6]) / 32768.0f * 2000.0f;
        s_latest.has_gyro = 1u;
        ++s_stats.frame_gyro;
        break;
    case WIT_FRAME_ANGLE:
        s_latest.roll_deg = (float)le_i16(&frame[2]) / 32768.0f * 180.0f;
        s_latest.pitch_deg = (float)le_i16(&frame[4]) / 32768.0f * 180.0f;
        s_latest.yaw_deg = (float)le_i16(&frame[6]) / 32768.0f * 180.0f;
        s_latest.has_angle = 1u;
        ++s_stats.frame_angle;
        break;
    default:
        ++s_stats.frame_other;
        return;
    }

    ++s_latest.sample_seq;
    s_stats.sample_seq = s_latest.sample_seq;
}

static uint8_t pop_rx_byte(uint8_t *data)
{
    if (s_rx_tail == s_rx_head) {
        return 0u;
    }

    *data = s_rx_buf[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1u) % WIT_RX_BUFFER_SIZE);
    return 1u;
}

void WitImuUart_Init(void)
{
    s_rx_head = 0u;
    s_rx_tail = 0u;
    memset(&s_latest, 0, sizeof(s_latest));

    DL_UART_Main_setRXFIFOThreshold(UART_IMU_INST, DL_UART_RX_FIFO_LEVEL_1_4_FULL);
    DL_UART_enableInterrupt(UART_IMU_INST,
                            DL_UART_INTERRUPT_RX |
                            DL_UART_INTERRUPT_RX_TIMEOUT_ERROR |
                            DL_UART_INTERRUPT_OVERRUN_ERROR);
    NVIC_EnableIRQ(UART_IMU_INST_INT_IRQN);
}

void WitImuUart_IrqHandler(void)
{
    DL_UART_IIDX pending;

    do {
        pending = DL_UART_getPendingInterrupt(UART_IMU_INST);

        switch (pending) {
        case DL_UART_IIDX_RX:
        case DL_UART_IIDX_RX_TIMEOUT_ERROR:
            while (!DL_UART_isRXFIFOEmpty(UART_IMU_INST)) {
                uint16_t next_head = (uint16_t)((s_rx_head + 1u) % WIT_RX_BUFFER_SIZE);
                uint8_t data = DL_UART_receiveData(UART_IMU_INST);

                ++s_stats.rx_bytes;
                if (next_head != s_rx_tail) {
                    s_rx_buf[s_rx_head] = data;
                    s_rx_head = next_head;
                } else {
                    ++s_stats.rx_overflow;
                }
            }
            break;
        case DL_UART_IIDX_OVERRUN_ERROR:
            ++s_stats.hw_overrun;
            DL_UART_clearInterruptStatus(UART_IMU_INST, DL_UART_INTERRUPT_OVERRUN_ERROR);
            while (!DL_UART_isRXFIFOEmpty(UART_IMU_INST)) {
                (void)DL_UART_receiveData(UART_IMU_INST);
            }
            break;
        default:
            break;
        }
    } while (pending != DL_UART_IIDX_NO_INTERRUPT);
}

uint8_t WitImuUart_ReadLatest(wit_imu_sample_t *sample)
{
    static uint8_t frame[WIT_FRAME_SIZE];
    static uint8_t frame_len;
    uint8_t data;

    while (pop_rx_byte(&data) != 0u) {
        if (frame_len == 0u) {
            if (data != WIT_FRAME_HEAD) {
                ++s_stats.sync_drop;
                continue;
            }
        } else if ((frame_len == 1u) &&
                   (data != WIT_FRAME_GYRO) &&
                   (data != WIT_FRAME_ANGLE)) {
            frame_len = 0u;
            ++s_stats.sync_drop;
            if (data == WIT_FRAME_HEAD) {
                frame[frame_len++] = data;
            }
            continue;
        }

        frame[frame_len++] = data;
        if (frame_len >= WIT_FRAME_SIZE) {
            parse_frame(frame);
            frame_len = 0u;
        }
    }

    if ((sample != NULL) && ((s_latest.has_angle != 0u) || (s_latest.has_gyro != 0u))) {
        *sample = s_latest;
        return 1u;
    }

    return 0u;
}

void WitImuUart_GetStats(wit_imu_uart_stats_t *stats)
{
    if (stats != NULL) {
        *stats = s_stats;
    }
}

void WitImuUart_ClearStats(void)
{
    memset((void *)&s_stats, 0, sizeof(s_stats));
}
