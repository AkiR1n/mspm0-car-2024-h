#include "signal_task.h"

#include "FreeRTOS.h"
#include "task.h"

#include "signal_driver.h"

#define SIGNAL_TASK_PERIOD_MS 10U

void signal_task(void *arg)
{
    TickType_t next;

    (void)arg;
    next = xTaskGetTickCount();

    signal_driver_init();

    for (;;) {
        signal_driver_update(SIGNAL_TASK_PERIOD_MS);
        vTaskDelayUntil(&next, pdMS_TO_TICKS(SIGNAL_TASK_PERIOD_MS));
    }
}
