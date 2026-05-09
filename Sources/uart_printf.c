
#include "uart_printf.h"
#include "bt_uart.h"

int uart_printf(const char *fmt, ...)
{
    static char buf[256];
    uint32_t i, len;
    va_list args;

    va_start(args,fmt);
    len = vsnprintf((char *)buf,sizeof(buf),(char *)fmt,args);
    va_end(args);

    if (len >= sizeof(buf)) {
        len = sizeof(buf) - 1U;
    }

    i = 0;
    while(i < len)
    {
        DL_UART_transmitData(UART0_INST,buf[i]);
        while(!DL_UART_isTXFIFOEmpty(UART0_INST));
        i++;
    }

    bt_uart_send((const uint8_t *)buf, len);

    return len;
}
