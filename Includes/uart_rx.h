#ifndef UART_RX_H_
#define UART_RX_H_

#include <stdint.h>

typedef struct {
    uint32_t rx_overflow;
    uint32_t hw_overrun;
} uart_rx_stats_t;

void uart_rx_init(void);
void uart_rx_irq_handler(void);
int uart_rx_get_char(char *ch);
void uart_rx_get_stats(uart_rx_stats_t *stats);
void uart_rx_clear_stats(void);

#endif
