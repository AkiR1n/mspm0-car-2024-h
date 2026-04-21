#include "ti_msp_dl_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "uart_printf.h"

#include "imu_task.h"
#include "motion_task.h"
#include "oled_task.h"
#include "log_task.h"

static void create_task_or_halt(TaskFunction_t fn,
                                const char *name,
                                configSTACK_DEPTH_TYPE stack_words,
                                UBaseType_t priority)
{
    if (xTaskCreate(fn, name, stack_words, NULL, priority, NULL) != pdPASS) {
        uart_printf("\r\n!!! xTaskCreate failed: %s !!!\r\n", name);
        for (;;) {
        }
    }
}

/*
 * configCHECK_FOR_STACK_OVERFLOW = 2 要求应用提供此钩子。
 * 首版调试阶段保留死循环 + 打印，实际板上可以断点或打 LED。
 */
void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    (void)task;
    uart_printf("\r\n!!! STACK OVERFLOW in task '%s' !!!\r\n", name);
    for (;;) { /* halt for inspection */ }
}

/*
 * mspm0-school-2026 entry
 *
 * 任务优先级（configMAX_PRIORITIES = 8）：
 *   6  motion    10 ms 周期
 *   5  imu       10 ms 周期
 *   2  oled      100 ms 刷新
 *   1  log       1 s 打印系统状态
 *
 * 栈大小（words，configSTACK_DEPTH_TYPE = size_t）：
 *   motion   512
 *   imu      512（DMP 初始化路径深）
 *   oled     384
 *   log      256
 *
 * 合计 ≈ 1664 words = 6.5 KB。FreeRTOS heap 16 KB（heap_4）承载 TCB + 栈。
 */
int main(void)
{
    SYSCFG_DL_init();

    create_task_or_halt(motion_task, "motion", 512, 6);
    create_task_or_halt(imu_task,    "imu",    512, 5);
    create_task_or_halt(oled_task,   "oled",   384, 2);
    create_task_or_halt(log_task,    "log",    256, 1);

    vTaskStartScheduler();
    uart_printf("\r\n!!! vTaskStartScheduler returned !!!\r\n");
    for (;;);
}
