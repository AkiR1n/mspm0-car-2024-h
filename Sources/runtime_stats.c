#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

void vMainConfigureRunTimeStats(void)
{
    /* 临时方案：直接使用 RTOS tick 作为运行时统计时基，不额外占用硬件定时器。 */
}

uint32_t ulMainGetRunTimeCounterValue(void)
{
    return (uint32_t)xTaskGetTickCountFromISR();
}
