#include "encoder_drv.h"

#include <stddef.h>

static encoder_t *s_encoders[2];

void Encoder_Init(encoder_t *encoder, const encoder_cfg_t *cfg)
{
    if ((encoder == NULL) || (cfg == NULL)) {
        return;
    }

    EncoderHal_Init();

    encoder->cfg = *cfg;
    if ((encoder->cfg.direction_sign > -1e-6f) &&
        (encoder->cfg.direction_sign < 1e-6f)) {
        encoder->cfg.direction_sign = 1.0f;
    }
    encoder->last_count = 0;
    encoder->count = 0;
    encoder->speed_rps = 0.0f;
    encoder->speed_mps = 0.0f;
    s_encoders[cfg->hal_id] = encoder;
}

void Encoder_OnEdgeIRQ(void)
{
    EncoderHal_HandleGpioIRQ();
}

void Encoder_OnSampleTick(void)
{
    EncoderHal_OnSampleTick();

    for (uint32_t i = 0; i < 2u; ++i) {
        float diff_count;
        float logical_count;
        float circumference_m;
        encoder_t *encoder = s_encoders[i];

        if (encoder == NULL) {
            continue;
        }

        encoder->count = EncoderHal_GetCount(encoder->cfg.hal_id);
        logical_count = (float)encoder->count * encoder->cfg.direction_sign;
        diff_count = logical_count - (float)encoder->last_count;
        encoder->last_count = (int32_t)logical_count;
        encoder->count = (int32_t)logical_count;

        if ((encoder->cfg.pulses_per_revolution == 0u) ||
            (encoder->cfg.sample_period_s <= 0.0f)) {
            encoder->speed_rps = 0.0f;
            encoder->speed_mps = 0.0f;
            continue;
        }

        encoder->speed_rps =
            diff_count /
            ((float)encoder->cfg.pulses_per_revolution * encoder->cfg.sample_period_s);
        circumference_m = 2.0f * 3.14159265f * encoder->cfg.wheel_radius_m;
        encoder->speed_mps = encoder->speed_rps * circumference_m;
    }
}

float Encoder_GetSpeedMps(const encoder_t *encoder)
{
    if (encoder == NULL) {
        return 0.0f;
    }
    return encoder->speed_mps;
}

float Encoder_GetSpeedRps(const encoder_t *encoder)
{
    if (encoder == NULL) {
        return 0.0f;
    }
    return encoder->speed_rps;
}

int32_t Encoder_GetCount(const encoder_t *encoder)
{
    if (encoder == NULL) {
        return 0;
    }
    return encoder->count;
}
