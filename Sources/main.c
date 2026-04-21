#include "ti_msp_dl_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "uart_printf.h"

#include "app_state.h"
#include "control_task.h"
#include "mode_debug_task.h"
#include "sensor_task.h"

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
 * 新主线：
 *   sensor_task     10 ms 采集编码器/IMU/循迹快照
 *   control_task    10 ms 执行双轮速度闭环与 Twist(v,w) 差速解算
 *   mode_debug_task 50 ms 维护命令、OLED 和串口调试输出
 */
int main(void)
{
    SYSCFG_DL_init();
    app_state_init();

    create_task_or_halt(sensor_task,     "sensor", 512, 6);
    create_task_or_halt(control_task,    "control", 512, 5);
    create_task_or_halt(mode_debug_task, "mode",   512, 2);

    vTaskStartScheduler();
    uart_printf("\r\n!!! vTaskStartScheduler returned !!!\r\n");
    for (;;);
}
