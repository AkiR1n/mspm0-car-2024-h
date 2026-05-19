#include "ti_msp_dl_config.h"
#include "interrupt.h"
#include "encoder_drv.h"
#include "uart_rx.h"
#include "bt_uart.h"
#include "wit_imu_uart.h"

/*
 * FreeRTOS SysTick 由 port 层接管；应用侧只保留 motion-v2 必需的编码器 ISR。
 */

void GROUP1_IRQHandler(void)
{
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
#if defined GPIO_ENCODER_PORT && defined GPIO_ENCODER_INT_IIDX
        case GPIO_ENCODER_INT_IIDX: {
            uint32_t encoder_pins =
                GPIO_ENCODER_PIN_A1_PIN | GPIO_ENCODER_PIN_A2_PIN |
                GPIO_ENCODER_PIN_B1_PIN | GPIO_ENCODER_PIN_B2_PIN;
            if (DL_GPIO_getEnabledInterruptStatus(GPIO_ENCODER_PORT, encoder_pins)) {
                Encoder_OnEdgeIRQ();
            }
            break;
        }
#endif
        default:
            break;
    }
}

#if defined TIMER_CALC_INST
void TIMA1_IRQHandler(void)
{
    DL_TimerA_clearInterruptStatus(TIMER_CALC_INST, DL_TIMERA_INTERRUPT_ZERO_EVENT);
    Encoder_OnSampleTick();
}
#endif

void UART0_IRQHandler(void)
{
    uart_rx_irq_handler();
}

void UART1_IRQHandler(void)
{
    bt_uart_irq_handler();
}

void UART2_IRQHandler(void)
{
    WitImuUart_IrqHandler();
}
