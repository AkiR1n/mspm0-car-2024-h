#include "clock.h"

/* 原裸机实现是忙等循环，FreeRTOS 下改成 vTaskDelay 让出 CPU。
 * 若在调度器启动前误调用，则直接返回，避免在 main() 初始化阶段卡死。 */
int mspm0_delay_ms(unsigned long num_ms)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        return 0;
    }
    vTaskDelay(pdMS_TO_TICKS(num_ms));
    return 0;
}

int mspm0_get_clock_ms(unsigned long *count)
{
    if (!count) return 1;
    *count = tick_ms;
    return 0;
}
