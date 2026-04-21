#include "encoder_hal.h"

#include "ti_msp_dl_config.h"

static volatile int32_t s_count[2];
static volatile uint32_t s_gpio_irq_count;
static volatile uint32_t s_sample_tick_count;

#define READ_BIT(port, pin) \
    ((DL_GPIO_readPins((port), (pin)) == (pin)) ? 0 : 1)

#define READ_A1 READ_BIT(GPIO_ENCODER_PORT, GPIO_ENCODER_PIN_A1_PIN)
#define READ_A2 READ_BIT(GPIO_ENCODER_PORT, GPIO_ENCODER_PIN_A2_PIN)
#define READ_B1 READ_BIT(GPIO_ENCODER_PORT, GPIO_ENCODER_PIN_B1_PIN)
#define READ_B2 READ_BIT(GPIO_ENCODER_PORT, GPIO_ENCODER_PIN_B2_PIN)

void EncoderHal_Init(void)
{
    DL_GPIO_initDigitalInputFeatures(GPIO_ENCODER_PIN_A1_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(GPIO_ENCODER_PIN_A2_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(GPIO_ENCODER_PIN_B1_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(GPIO_ENCODER_PIN_B2_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);

    s_count[ENCODER_HAL_LEFT] = 0;
    s_count[ENCODER_HAL_RIGHT] = 0;
    s_gpio_irq_count = 0u;
    s_sample_tick_count = 0u;

    DL_GPIO_clearInterruptStatus(GPIO_ENCODER_PORT,
        GPIO_ENCODER_PIN_A1_PIN | GPIO_ENCODER_PIN_A2_PIN |
        GPIO_ENCODER_PIN_B1_PIN | GPIO_ENCODER_PIN_B2_PIN);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
}

void EncoderHal_HandleGpioIRQ(void)
{
    const uint32_t pins =
        GPIO_ENCODER_PIN_A1_PIN | GPIO_ENCODER_PIN_A2_PIN |
        GPIO_ENCODER_PIN_B1_PIN | GPIO_ENCODER_PIN_B2_PIN;
    const uint32_t status =
        DL_GPIO_getEnabledInterruptStatus(GPIO_ENCODER_PORT, pins);

    if (status != 0u) {
        ++s_gpio_irq_count;
    }

    if (status & GPIO_ENCODER_PIN_A1_PIN) {
        DL_GPIO_clearInterruptStatus(GPIO_ENCODER_PORT, GPIO_ENCODER_PIN_A1_PIN);
        s_count[ENCODER_HAL_LEFT] += (READ_A1 ^ READ_A2) ? 1 : -1;
    }
    if (status & GPIO_ENCODER_PIN_A2_PIN) {
        DL_GPIO_clearInterruptStatus(GPIO_ENCODER_PORT, GPIO_ENCODER_PIN_A2_PIN);
        s_count[ENCODER_HAL_LEFT] += (READ_A1 ^ READ_A2) ? -1 : 1;
    }
    if (status & GPIO_ENCODER_PIN_B1_PIN) {
        DL_GPIO_clearInterruptStatus(GPIO_ENCODER_PORT, GPIO_ENCODER_PIN_B1_PIN);
        s_count[ENCODER_HAL_RIGHT] += (READ_B1 ^ READ_B2) ? 1 : -1;
    }
    if (status & GPIO_ENCODER_PIN_B2_PIN) {
        DL_GPIO_clearInterruptStatus(GPIO_ENCODER_PORT, GPIO_ENCODER_PIN_B2_PIN);
        s_count[ENCODER_HAL_RIGHT] += (READ_B1 ^ READ_B2) ? -1 : 1;
    }
}

int32_t EncoderHal_GetCount(encoder_hal_id_t id)
{
    return s_count[id];
}

uint8_t EncoderHal_GetRawA1(void)
{
    return (DL_GPIO_readPins(GPIO_ENCODER_PORT, GPIO_ENCODER_PIN_A1_PIN) ==
            GPIO_ENCODER_PIN_A1_PIN)
        ? 1u
        : 0u;
}

uint8_t EncoderHal_GetRawA2(void)
{
    return (DL_GPIO_readPins(GPIO_ENCODER_PORT, GPIO_ENCODER_PIN_A2_PIN) ==
            GPIO_ENCODER_PIN_A2_PIN)
        ? 1u
        : 0u;
}

uint8_t EncoderHal_GetRawB1(void)
{
    return (DL_GPIO_readPins(GPIO_ENCODER_PORT, GPIO_ENCODER_PIN_B1_PIN) ==
            GPIO_ENCODER_PIN_B1_PIN)
        ? 1u
        : 0u;
}

uint8_t EncoderHal_GetRawB2(void)
{
    return (DL_GPIO_readPins(GPIO_ENCODER_PORT, GPIO_ENCODER_PIN_B2_PIN) ==
            GPIO_ENCODER_PIN_B2_PIN)
        ? 1u
        : 0u;
}

uint32_t EncoderHal_GetGpioIrqCount(void)
{
    return s_gpio_irq_count;
}

uint32_t EncoderHal_GetSampleTickCount(void)
{
    return s_sample_tick_count;
}

void EncoderHal_OnSampleTick(void)
{
    ++s_sample_tick_count;
}
