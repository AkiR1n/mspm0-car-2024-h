#include "ti_msp_dl_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "uart_printf.h"

#include "app_state.h"
#include "control_task.h"
#include "main_task.h"
#include "signal_driver.h"
#include "signal_task.h"
#include "oled_task.h"
#include "sensor_task.h"
#include "test_task.h"

static void configure_uart_rx_idle_pullups(void)
{
    DL_GPIO_initPeripheralInputFunctionFeatures(
        GPIO_UART0_IOMUX_RX,
        GPIO_UART0_IOMUX_RX_FUNC,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(
        GPIO_UART_DBG_IOMUX_RX,
        GPIO_UART_DBG_IOMUX_RX_FUNC,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
}

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
 *   main_task       10 ms 题目主流程入口
 *   test_task       50 ms 测试、调参、串口调试入口
 *   oled_task      100 ms 显示当前题目/阶段/姿态
 */
int main(void)
{
    SYSCFG_DL_init();
    configure_uart_rx_idle_pullups();
    app_state_init();
    signal_driver_init();

    create_task_or_halt(sensor_task,  "sensor",  512, 6);
    create_task_or_halt(control_task, "control", 512, 5);
    create_task_or_halt(main_task,    "main",    512, 3);
    create_task_or_halt(signal_task,  "signal",  256, 1);
    create_task_or_halt(test_task,    "test",    512, 2);
    create_task_or_halt(oled_task,    "oled",    384, 1);

    vTaskStartScheduler();
    uart_printf("\r\n!!! vTaskStartScheduler returned !!!\r\n");
    for (;;);
}
