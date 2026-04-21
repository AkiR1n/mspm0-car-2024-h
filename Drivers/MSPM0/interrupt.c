#include "ti_msp_dl_config.h"
#include "interrupt.h"
#include "encoder.h"

/*
 * FreeRTOS SysTick 由 port 层接管；应用侧只保留 motion-v2 必需的编码器 ISR。
 */

void GROUP1_IRQHandler(void)
{
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
#if defined GPIO_ENCODER_PORT && defined GPIO_MULTIPLE_GPIOB_INT_IIDX
        case GPIO_MULTIPLE_GPIOB_INT_IIDX: {
            uint32_t encoder_pins =
                GPIO_ENCODER_PIN_A1_PIN | GPIO_ENCODER_PIN_A2_PIN |
                GPIO_ENCODER_PIN_B1_PIN | GPIO_ENCODER_PIN_B2_PIN;
            if (DL_GPIO_getEnabledInterruptStatus(GPIO_ENCODER_PORT, encoder_pins)) {
                encoder_on_gpio_irq();
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
    DL_TimerA_clearInterruptStatus(TIMER_CALC_INST, DL_TIMER_IIDX_ZERO);
    encoder_on_tick_irq();
}
#endif
