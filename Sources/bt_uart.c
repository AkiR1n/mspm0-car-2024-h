#include "bt_uart.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "ti_msp_dl_config.h"

#define BT_RX_BUFFER_SIZE 128u
#define BT_TX_BUFFER_SIZE 1024u

static volatile uint8_t s_rx_buf[BT_RX_BUFFER_SIZE];
static volatile uint32_t s_rx_head = 0u;
static volatile uint32_t s_rx_tail = 0u;

static volatile uint8_t s_tx_buf[BT_TX_BUFFER_SIZE];
static volatile uint32_t s_tx_head = 0u;
static volatile uint32_t s_tx_tail = 0u;
static volatile uint8_t s_tx_active = 0u;

static volatile uint8_t s_initialized = 0u;

void bt_uart_init(void)
{
    if (s_initialized != 0u) {
        return;
    }

    s_rx_head = 0u;
    s_rx_tail = 0u;
    s_tx_head = 0u;
    s_tx_tail = 0u;
    s_tx_active = 0u;

    DL_UART_Main_setRXFIFOThreshold(UART_DBG_INST, DL_UART_RX_FIFO_LEVEL_1_4_FULL);
    DL_UART_enableInterrupt(UART_DBG_INST,
                            DL_UART_INTERRUPT_RX |
                            DL_UART_INTERRUPT_RX_TIMEOUT_ERROR |
                            DL_UART_INTERRUPT_OVERRUN_ERROR);
    NVIC_EnableIRQ(UART_DBG_INST_INT_IRQN);
    s_initialized = 1u;
}

void bt_uart_irq_handler(void)
{
    DL_UART_IIDX pending;

    do {
        pending = DL_UART_getPendingInterrupt(UART_DBG_INST);

        switch (pending) {
        case DL_UART_IIDX_RX:
        case DL_UART_IIDX_RX_TIMEOUT_ERROR:
            while (!DL_UART_isRXFIFOEmpty(UART_DBG_INST)) {
                uint32_t next_head = (s_rx_head + 1u) % BT_RX_BUFFER_SIZE;
                uint8_t data = DL_UART_receiveData(UART_DBG_INST);

                if (next_head != s_rx_tail) {
                    s_rx_buf[s_rx_head] = data;
                    s_rx_head = next_head;
                }
            }
            break;
        case DL_UART_IIDX_OVERRUN_ERROR:
            DL_UART_clearInterruptStatus(UART_DBG_INST, DL_UART_INTERRUPT_OVERRUN_ERROR);
            while (!DL_UART_isRXFIFOEmpty(UART_DBG_INST)) {
                (void)DL_UART_receiveData(UART_DBG_INST);
            }
            break;
        case DL_UART_IIDX_TX:
            while (!DL_UART_isTXFIFOFull(UART_DBG_INST)) {
                if (s_tx_tail == s_tx_head) {
                    DL_UART_disableInterrupt(UART_DBG_INST, DL_UART_INTERRUPT_TX);
                    s_tx_active = 0u;
                    break;
                }
                DL_UART_transmitData(UART_DBG_INST, s_tx_buf[s_tx_tail]);
                s_tx_tail = (s_tx_tail + 1u) % BT_TX_BUFFER_SIZE;
            }
            break;
        default:
            break;
        }
    } while (pending != DL_UART_IIDX_NO_INTERRUPT);
}

int bt_uart_get_char(char *ch)
{
    if ((ch == NULL) || (s_rx_head == s_rx_tail)) {
        return 0;
    }

    *ch = (char)s_rx_buf[s_rx_tail];
    s_rx_tail = (s_rx_tail + 1u) % BT_RX_BUFFER_SIZE;
    return 1;
}

void bt_uart_send(const uint8_t *data, uint32_t len)
{
    if ((s_initialized == 0u) || (len == 0u)) {
        return;
    }

    for (uint32_t i = 0u; i < len; i++) {
        DL_UART_transmitDataBlocking(UART_DBG_INST, data[i]);
    }
}

void bt_uart_send_str(const char *str)
{
    bt_uart_send((const uint8_t *)str, (uint32_t)strlen(str));
}

int bt_printf(const char *fmt, ...)
{
    static char buf[256];
    va_list args;
    int len;

    va_start(args, fmt);
    len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len < 0) {
        return 0;
    }
    if ((uint32_t)len >= sizeof(buf)) {
        len = (int)(sizeof(buf) - 1u);
    }

    bt_uart_send((const uint8_t *)buf, (uint32_t)len);
    return len;
}
