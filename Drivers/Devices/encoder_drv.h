#ifndef ENCODER_DRV_H_
#define ENCODER_DRV_H_

#include <stddef.h>
#include <stdint.h>

#include "encoder_hal.h"

typedef struct {
    encoder_hal_id_t hal_id;
    uint32_t         pulses_per_revolution;
    float            wheel_radius_m;
    float            sample_period_s;
    float            direction_sign;
} encoder_cfg_t;

typedef struct {
    encoder_cfg_t cfg;
    int32_t       last_count;
    int32_t       count;
    float         speed_rps;
    float         speed_mps;
} encoder_t;

void Encoder_Init(encoder_t *encoder, const encoder_cfg_t *cfg);
void Encoder_OnEdgeIRQ(void);
void Encoder_OnSampleTick(void);
float Encoder_GetSpeedMps(const encoder_t *encoder);
float Encoder_GetSpeedRps(const encoder_t *encoder);
int32_t Encoder_GetCount(const encoder_t *encoder);

#endif
