#ifndef _CLOCK_H_
#define _CLOCK_H_

#include <stdint.h>
#include <cmsis_gcc.h>
#include "FreeRTOS.h"
#include "task.h"

/* FreeRTOS 版：tick_ms 从内核 tick 派生。
 * 若在 ISR 中读取，则自动改走 xTaskGetTickCountFromISR()，避免 turn_detection
 * 这类中断上下文路径误调用普通 API。 */
static inline unsigned long mspm0_now_ms(void)
{
    TickType_t ticks;

    if (__get_IPSR() != 0U) {
        ticks = xTaskGetTickCountFromISR();
    } else {
        ticks = xTaskGetTickCount();
    }

    return (unsigned long)(ticks * portTICK_PERIOD_MS);
}

#define tick_ms  (mspm0_now_ms())

int mspm0_delay_ms(unsigned long num_ms);
int mspm0_get_clock_ms(unsigned long *count);

#endif  /* _CLOCK_H_ */
