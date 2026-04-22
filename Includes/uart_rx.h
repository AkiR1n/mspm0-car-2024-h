#ifndef UART_RX_H_
#define UART_RX_H_

#include <stdint.h>

void uart_rx_init(void);
void uart_rx_irq_handler(void);
int uart_rx_get_char(char *ch);

#endif
