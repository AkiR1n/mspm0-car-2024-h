
#include "uart_printf.h"
#include "bt_uart.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

int uart_printf(const char *fmt, ...)
{
    static char buf[256];
    static SemaphoreHandle_t s_uart_printf_mutex = NULL;
    uint32_t i, len;
    va_list args;
    BaseType_t locked = pdFALSE;

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        if (s_uart_printf_mutex == NULL) {
            s_uart_printf_mutex = xSemaphoreCreateMutex();
        }
        if (s_uart_printf_mutex != NULL) {
            locked = xSemaphoreTake(s_uart_printf_mutex, pdMS_TO_TICKS(50));
        }
        if (locked != pdTRUE) {
            return 0;
        }
    }

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

    if ((locked == pdTRUE) && (s_uart_printf_mutex != NULL)) {
        xSemaphoreGive(s_uart_printf_mutex);
    }

    return len;
}
