#ifndef ENCODER_HAL_H_
#define ENCODER_HAL_H_

#include <stdint.h>

typedef enum {
    ENCODER_HAL_LEFT = 0,
    ENCODER_HAL_RIGHT = 1,
} encoder_hal_id_t;

void EncoderHal_Init(void);
void EncoderHal_HandleGpioIRQ(void);
int32_t EncoderHal_GetCount(encoder_hal_id_t id);
uint8_t EncoderHal_GetRawA1(void);
uint8_t EncoderHal_GetRawA2(void);
uint8_t EncoderHal_GetRawB1(void);
uint8_t EncoderHal_GetRawB2(void);
uint32_t EncoderHal_GetGpioIrqCount(void);
uint32_t EncoderHal_GetSampleTickCount(void);
void EncoderHal_OnSampleTick(void);

#endif
