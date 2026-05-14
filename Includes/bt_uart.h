#ifndef BT_UART_H_
#define BT_UART_H_

#include <stdint.h>

typedef struct {
    uint32_t rx_overflow;
    uint32_t hw_overrun;
} bt_uart_stats_t;

void bt_uart_init(void);
void bt_uart_irq_handler(void);
int bt_uart_get_char(char *ch);
void bt_uart_send(const uint8_t *data, uint32_t len);
void bt_uart_send_str(const char *str);
int bt_printf(const char *fmt, ...);
void bt_uart_get_stats(bt_uart_stats_t *stats);
void bt_uart_clear_stats(void);

#endif
