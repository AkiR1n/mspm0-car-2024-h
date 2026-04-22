#include "uart_rx.h"

#include "ti_msp_dl_config.h"

#define UART_RX_BUFFER_SIZE 128u

static volatile uint8_t s_rx_buf[UART_RX_BUFFER_SIZE];
static volatile uint32_t s_rx_head = 0u;
static volatile uint32_t s_rx_tail = 0u;
static volatile uint8_t s_rx_initialized = 0u;

void uart_rx_init(void)
{
    if (s_rx_initialized != 0u) {
        return;
    }

    s_rx_head = 0u;
    s_rx_tail = 0u;
    DL_UART_Main_setRXFIFOThreshold(UART0_INST, DL_UART_RX_FIFO_LEVEL_1_4_FULL);
    DL_UART_enableInterrupt(UART0_INST,
                            DL_UART_INTERRUPT_RX |
                            DL_UART_INTERRUPT_RX_TIMEOUT_ERROR |
                            DL_UART_INTERRUPT_OVERRUN_ERROR);
    NVIC_EnableIRQ(UART0_INST_INT_IRQN);
    s_rx_initialized = 1u;
}

void uart_rx_irq_handler(void)
{
    DL_UART_IIDX pending;

    do {
        pending = DL_UART_getPendingInterrupt(UART0_INST);

        switch (pending) {
        case DL_UART_IIDX_RX:
        case DL_UART_IIDX_RX_TIMEOUT_ERROR:
            while (!DL_UART_isRXFIFOEmpty(UART0_INST)) {
                uint32_t next_head = (s_rx_head + 1u) % UART_RX_BUFFER_SIZE;
                uint8_t data = DL_UART_receiveData(UART0_INST);

                if (next_head != s_rx_tail) {
                    s_rx_buf[s_rx_head] = data;
                    s_rx_head = next_head;
                }
            }
            break;
        case DL_UART_IIDX_OVERRUN_ERROR:
            DL_UART_clearInterruptStatus(UART0_INST, DL_UART_INTERRUPT_OVERRUN_ERROR);
            while (!DL_UART_isRXFIFOEmpty(UART0_INST)) {
                (void)DL_UART_receiveData(UART0_INST);
            }
            break;
        default:
            break;
        }
    } while (pending != DL_UART_IIDX_NO_INTERRUPT);
}

int uart_rx_get_char(char *ch)
{
    if ((ch == NULL) || (s_rx_head == s_rx_tail)) {
        return 0;
    }

    *ch = (char)s_rx_buf[s_rx_tail];
    s_rx_tail = (s_rx_tail + 1u) % UART_RX_BUFFER_SIZE;
    return 1;
}
