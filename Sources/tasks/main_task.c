#include "main_task.h"

#include <math.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app_state.h"
#include "chassis_system.h"

#define MAIN_TASK_PERIOD_MS               10U
#define MAIN_DT_S                         0.01f
#define MAIN_PI                           3.14159265f
#define MAIN_WHEEL_RADIUS_M               0.0325f
#define MAIN_ENCODER_PPR                  (13.0f * 28.0f * 4.0f)
#define MAIN_M_PER_COUNT                  ((2.0f * MAIN_PI * MAIN_WHEEL_RADIUS_M) / MAIN_ENCODER_PPR)
#define MAIN_ALIGN_SPEED_THRESHOLD_MPS    0.03f
#define MAIN_ALIGN_STABLE_MS              400U
#define MAIN_GAP_CRUISE_SPEED_MPS         0.68f
#define MAIN_GAP_END_SPEED_MPS            0.40f
#define MAIN_GAP_D_EXIT_SPEED_MPS         0.30f
#define MAIN_GAP_D_CENTER_CONFIRM_MS      60U
#define MAIN_GAP_D_CENTER_POS_MAX         10
#define MAIN_GAP_BD_D_CONFIRM_MS          30U
#define MAIN_GAP_BD_D_POS_MAX             24
#define MAIN_GAP_D_CENTER_W_SCALE         0.24f
#define MAIN_GAP_D_CENTER_W_LIMIT_RADPS   0.35f
#define MAIN_GAP_ARC_FALLBACK_MARGIN_M    0.08f
#define MAIN_GAP_BD_ARC_FALLBACK_MARGIN_M (-0.02f)
#define MAIN_GAP_ARC_FALLBACK_SPEED_MPS   0.20f
#define MAIN_GAP_W_LIMIT_RADPS            1.70f
#define MAIN_BASIC_STRAIGHT_DEFAULT_DISTANCE_M 0.50f
#define MAIN_BASIC_STRAIGHT_DEFAULT_SPEED_MPS  0.25f
#define MAIN_BASIC_STRAIGHT_END_SPEED_MPS      0.10f
#define MAIN_BASIC_LINE_DEFAULT_SPEED_MPS      0.18f
#define MAIN_BASIC_LINE_SEARCH_SPEED_MPS       0.08f
#define MAIN_BASIC_MIN_DISTANCE_M              0.05f
#define MAIN_BASIC_MAX_DISTANCE_M              2.50f
#define MAIN_BASIC_MIN_SPEED_MPS               0.05f
#define MAIN_BASIC_MAX_SPEED_MPS               0.70f
#define MAIN_BASIC_LINE_W_LIMIT_RADPS          2.20f
// Positive line controller output currently steers opposite to the installed sensor/chassis orientation.
#define MAIN_LINE_STEER_SIGN                   (-1.0f)
// Positive chassis w produces negative IMU yaw on the current hardware.
#define MAIN_HEADING_STEER_SIGN                (-1.0f)
#define MAIN_ARC_W_LIMIT_RADPS            1.80f
#define MAIN_ARC_SEARCH_SPEED_MPS         0.12f
#define MAIN_ARC_SEARCH_W_RADPS           1.10f
#define MAIN_ARC_FAST_ERROR_THRESHOLD     8
#define MAIN_ARC_MEDIUM_ERROR_THRESHOLD   18
#define MAIN_ARC_MAX_ERROR_THRESHOLD      30
#define MAIN_ARC_EDGE_BOOST_THRESHOLD     18
#define MAIN_ARC_EDGE_BOOST_MAX_SCALE     1.15f
#define MAIN_ARC_ENTRY_HOLD_DISTANCE_M    0.10f
#define MAIN_ARC_ENTRY_RAMP_END_M         0.25f
#define MAIN_ARC_ENTRY_START_SPEED_MPS    0.28f
#define MAIN_ARC_ENTRY_DA_START_SPEED_MPS 0.24f
#define MAIN_ARC_ENTRY_W_LIMIT_RADPS      1.00f
#define MAIN_ARC_EXIT_HEADING_TOL_DEG     8.0f
#define MAIN_ARC_EXIT_OVERRUN_TOL_DEG     14.0f
#define MAIN_ARC_EXIT_ALIGN_SPEED_MPS     0.10f
#define MAIN_ARC_EXIT_ALIGN_W_LIMIT_RADPS 1.25f
#define MAIN_ARC_CB_EXIT_ALIGN_SPEED_MPS  0.24f
#define MAIN_ARC_CB_EXIT_ALIGN_W_LIMIT_RADPS 2.53f
#define MAIN_ARC_Q4_A_EXIT_ALIGN_SPEED_MPS 0.24f
#define MAIN_ARC_Q4_A_EXIT_ALIGN_W_LIMIT_RADPS 2.53f
#define MAIN_ARC_EXIT_MAX_OVERRUN_M       0.12f
#define MAIN_FINAL_A_ALIGN_START_M        1.15f
#define MAIN_FINAL_A_ALIGN_SPEED_MPS      0.18f
#define MAIN_FINAL_A_ALIGN_W_LIMIT_RADPS  0.35f
#define MAIN_FINAL_A_EXIT_MIN_DISTANCE_M  1.16f
#define MAIN_FINAL_A_EXIT_MAX_DISTANCE_M  1.34f
#define MAIN_FINAL_A_EXIT_YAW_MIN_DEG     155.0f
#define MAIN_FINAL_A_COMPENSATE_M         0.000f
#define MAIN_REACQUIRE_CONFIRM_MS         40U
#define MAIN_V_ACCEL_MPS2                 2.8f
#define MAIN_V_DECEL_MPS2                 2.2f
// Field geometry angles are math-positive; the installed WT101 yaw sign is opposite.
#define MAIN_FIELD_TO_IMU_YAW_SIGN        (-1.0f)
#define MAIN_HEADING_AB_DEG               0.0f
#define MAIN_HEADING_CD_DEG               180.0f
#define MAIN_HEADING_AC_INNER_BIAS_DEG    (1.0f)
#define MAIN_HEADING_AC_DEG               (-38.659809f + MAIN_HEADING_AC_INNER_BIAS_DEG)
// Inward bias on B->D after WT101 yaw sign conversion.
#define MAIN_HEADING_BD_INNER_BIAS_DEG    (0.0f)
#define MAIN_HEADING_BD_DEG               (-141.340191f + MAIN_HEADING_BD_INNER_BIAS_DEG)

typedef struct {
    float    expected_length_m;
    float    min_exit_m;
    float    front_end_m;
    float    rear_start_m;
    float    speed_front_mps;
    float    speed_mid_mps;
    float    speed_rear_mps;
    float    turn_scale_front;
    float    turn_scale_mid;
    float    turn_scale_rear;
    float    inner_bias_start_m;
    float    inner_bias_full_m;
    float    inner_bias_w_radps;
    float    edge_boost_max_scale;
    float    w_limit_radps;
    uint16_t lost_confirm_ms;
    int8_t   inner_bias_sign;
} arc_profile_t;

typedef struct {
    app_challenge_phase_t phase;
    app_phase_action_t    action;
    float                 geometry_heading_deg;
    float                 nominal_distance_m;
    float                 min_exit_m;
    app_event_id_t        complete_event;
    const arc_profile_t  *arc_profile;
} phase_descriptor_t;

typedef struct {
    uint8_t          active;
    app_mode_t       active_mode;
    app_challenge_t  challenge_active;
    uint8_t          sequence_index;
    uint8_t          lap_index;
    uint8_t          lap_total;
    uint8_t          checkpoint_count;
    uint16_t         align_stable_ms;
    uint16_t         line_seen_ms;
    uint16_t         line_missing_ms;
    uint8_t          gap_left_line;
    uint8_t          arc_has_seen_line;
    int32_t          phase_start_left_count;
    int32_t          phase_start_right_count;
    float            arc_last_error;
    float            current_v_mps;
    float            phase_distance_m;
    float            target_distance_m;
    float            cruise_speed_mps;
    float            arc_entry_yaw_deg;
    float            final_a_comp_start_distance_m;
    float            field_yaw_offset_deg;
    float            geometry_heading_deg;
    float            hold_heading_deg;
    float            heading_error_deg;
    float            target_speed_mps;
    uint8_t          final_a_compensating;
    app_main_state_t state;
} main_task_ctx_t;

static const arc_profile_t k_arc_profile_bc = {
    .expected_length_m = 1.257f,
    .min_exit_m = 1.08f,
    .front_end_m = 0.32f,
    .rear_start_m = 0.86f,
    .speed_front_mps = 0.74f,
    .speed_mid_mps = 0.60f,
    .speed_rear_mps = 0.39f,
    .turn_scale_front = 1.00f,
    .turn_scale_mid = 1.08f,
    .turn_scale_rear = 1.20f,
    .inner_bias_start_m = 0.70f,
    .inner_bias_full_m = 0.98f,
    .inner_bias_w_radps = 0.30f,
    .edge_boost_max_scale = MAIN_ARC_EDGE_BOOST_MAX_SCALE,
    .w_limit_radps = MAIN_ARC_W_LIMIT_RADPS,
    .lost_confirm_ms = 70u,
    .inner_bias_sign = -1,
};

static const arc_profile_t k_arc_profile_cb = {
    .expected_length_m = 1.257f,
    .min_exit_m = 1.07f,
    .front_end_m = 0.28f,
    .rear_start_m = 0.88f,
    .speed_front_mps = 0.62f,
    .speed_mid_mps = 0.52f,
    .speed_rear_mps = 0.34f,
    .turn_scale_front = 0.94f,
    .turn_scale_mid = 1.02f,
    .turn_scale_rear = 1.10f,
    .inner_bias_start_m = 0.82f,
    .inner_bias_full_m = 1.06f,
    .inner_bias_w_radps = 0.18f,
    .edge_boost_max_scale = 1.06f,
    .w_limit_radps = 1.50f,
    .lost_confirm_ms = 90u,
    .inner_bias_sign = 1,
};

static const arc_profile_t k_arc_profile_da_q2 = {
    .expected_length_m = 1.257f,
    .min_exit_m = 1.05f,
    .front_end_m = 0.30f,
    .rear_start_m = 0.90f,
    .speed_front_mps = 0.72f,
    .speed_mid_mps = 0.62f,
    .speed_rear_mps = 0.41f,
    .turn_scale_front = 1.00f,
    .turn_scale_mid = 1.05f,
    .turn_scale_rear = 1.14f,
    .inner_bias_start_m = 0.76f,
    .inner_bias_full_m = 1.04f,
    .inner_bias_w_radps = 0.22f,
    .edge_boost_max_scale = MAIN_ARC_EDGE_BOOST_MAX_SCALE,
    .w_limit_radps = MAIN_ARC_W_LIMIT_RADPS,
    .lost_confirm_ms = 60u,
    .inner_bias_sign = 1,
};

static const arc_profile_t k_arc_profile_da_q3q4 = {
    .expected_length_m = 1.257f,
    .min_exit_m = 1.05f,
    .front_end_m = 0.30f,
    .rear_start_m = 0.90f,
    .speed_front_mps = 0.62f,
    .speed_mid_mps = 0.52f,
    .speed_rear_mps = 0.34f,
    .turn_scale_front = 0.92f,
    .turn_scale_mid = 1.00f,
    .turn_scale_rear = 1.08f,
    .inner_bias_start_m = 0.84f,
    .inner_bias_full_m = 1.08f,
    .inner_bias_w_radps = 0.14f,
    .edge_boost_max_scale = 1.06f,
    .w_limit_radps = 1.50f,
    .lost_confirm_ms = 90u,
    .inner_bias_sign = 1,
};

static const phase_descriptor_t k_q1_sequence[] = {
    {APP_CHALLENGE_PHASE_ALIGN_A_TO_B, APP_PHASE_ACTION_ALIGN_START,  MAIN_HEADING_AB_DEG, 0.0f,   0.0f,   APP_EVENT_NONE,   NULL},
    {APP_CHALLENGE_PHASE_GAP_AB,       APP_PHASE_ACTION_GAP_TRAVERSE, MAIN_HEADING_AB_DEG, 1.000f, 0.78f,  APP_EVENT_PASS_B, NULL},
    {APP_CHALLENGE_PHASE_STOP_B,       APP_PHASE_ACTION_STOP_AND_SIGNAL, 0.0f,              0.0f,   0.0f,   APP_EVENT_STOP,   NULL},
};

static const phase_descriptor_t k_q2_sequence[] = {
    {APP_CHALLENGE_PHASE_ALIGN_A_TO_B, APP_PHASE_ACTION_ALIGN_START,  MAIN_HEADING_AB_DEG, 0.0f,   0.0f,   APP_EVENT_NONE,   NULL},
    {APP_CHALLENGE_PHASE_GAP_AB,       APP_PHASE_ACTION_GAP_TRAVERSE, MAIN_HEADING_AB_DEG, 1.000f, 0.78f,  APP_EVENT_PASS_B, NULL},
    {APP_CHALLENGE_PHASE_ARC_BC,       APP_PHASE_ACTION_ARC_TRACK,    0.0f,                 0.0f,   0.0f,   APP_EVENT_PASS_C, &k_arc_profile_bc},
    {APP_CHALLENGE_PHASE_GAP_CD,       APP_PHASE_ACTION_GAP_TRAVERSE, MAIN_HEADING_CD_DEG, 1.000f, 0.78f,  APP_EVENT_PASS_D, NULL},
    {APP_CHALLENGE_PHASE_ARC_DA,       APP_PHASE_ACTION_ARC_TRACK,    0.0f,                 0.0f,   0.0f,   APP_EVENT_PASS_A, &k_arc_profile_da_q2},
    {APP_CHALLENGE_PHASE_STOP_A,       APP_PHASE_ACTION_STOP_AND_SIGNAL, 0.0f,              0.0f,   0.0f,   APP_EVENT_STOP,   NULL},
};

static const phase_descriptor_t k_q3_sequence[] = {
    {APP_CHALLENGE_PHASE_ALIGN_A_TO_C, APP_PHASE_ACTION_ALIGN_START,  MAIN_HEADING_AC_DEG, 0.0f,   0.0f,   APP_EVENT_NONE,   NULL},
    {APP_CHALLENGE_PHASE_GAP_AC,       APP_PHASE_ACTION_GAP_TRAVERSE, MAIN_HEADING_AC_DEG, 1.281f, 1.03f,  APP_EVENT_PASS_C, NULL},
    {APP_CHALLENGE_PHASE_ARC_CB,       APP_PHASE_ACTION_ARC_TRACK,    0.0f,                 0.0f,   0.0f,   APP_EVENT_PASS_B, &k_arc_profile_cb},
    {APP_CHALLENGE_PHASE_GAP_BD,       APP_PHASE_ACTION_GAP_TRAVERSE, MAIN_HEADING_BD_DEG, 1.281f, 1.03f,  APP_EVENT_PASS_D, NULL},
    {APP_CHALLENGE_PHASE_ARC_DA,       APP_PHASE_ACTION_ARC_TRACK,    0.0f,                 0.0f,   0.0f,   APP_EVENT_PASS_A, &k_arc_profile_da_q3q4},
    {APP_CHALLENGE_PHASE_STOP_A,       APP_PHASE_ACTION_STOP_AND_SIGNAL, 0.0f,              0.0f,   0.0f,   APP_EVENT_STOP,   NULL},
};

static const phase_descriptor_t k_q4_sequence[] = {
    {APP_CHALLENGE_PHASE_ALIGN_A_TO_C, APP_PHASE_ACTION_ALIGN_START,  MAIN_HEADING_AC_DEG, 0.0f,   0.0f,   APP_EVENT_NONE,   NULL},
    {APP_CHALLENGE_PHASE_GAP_AC,       APP_PHASE_ACTION_GAP_TRAVERSE, MAIN_HEADING_AC_DEG, 1.281f, 1.03f,  APP_EVENT_PASS_C, NULL},
    {APP_CHALLENGE_PHASE_ARC_CB,       APP_PHASE_ACTION_ARC_TRACK,    0.0f,                 0.0f,   0.0f,   APP_EVENT_PASS_B, &k_arc_profile_cb},
    {APP_CHALLENGE_PHASE_GAP_BD,       APP_PHASE_ACTION_GAP_TRAVERSE, MAIN_HEADING_BD_DEG, 1.281f, 1.03f,  APP_EVENT_PASS_D, NULL},
    {APP_CHALLENGE_PHASE_ARC_DA,       APP_PHASE_ACTION_ARC_TRACK,    0.0f,                 0.0f,   0.0f,   APP_EVENT_PASS_A, &k_arc_profile_da_q3q4},
    {APP_CHALLENGE_PHASE_STOP_A,       APP_PHASE_ACTION_STOP_AND_SIGNAL, 0.0f,              0.0f,   0.0f,   APP_EVENT_STOP,   NULL},
};

static float clampf(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float absf(float value)
{
    if (value < 0.0f) {
        return -value;
    }
    return value;
}

static int16_t absi16(int16_t value)
{
    if (value < 0) {
        return (int16_t)-value;
    }
    return value;
}

static float lerpf(float start, float end, float t)
{
    return start + ((end - start) * t);
}

static float apply_line_steer_sign(float w_radps);

static float wrap_angle_deg(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static const phase_descriptor_t *get_sequence(app_challenge_t challenge, uint8_t *count)
{
    switch (challenge) {
    case APP_CHALLENGE_Q1:
        if (count != NULL) {
            *count = (uint8_t)(sizeof(k_q1_sequence) / sizeof(k_q1_sequence[0]));
        }
        return k_q1_sequence;
    case APP_CHALLENGE_Q2:
        if (count != NULL) {
            *count = (uint8_t)(sizeof(k_q2_sequence) / sizeof(k_q2_sequence[0]));
        }
        return k_q2_sequence;
    case APP_CHALLENGE_Q3:
        if (count != NULL) {
            *count = (uint8_t)(sizeof(k_q3_sequence) / sizeof(k_q3_sequence[0]));
        }
        return k_q3_sequence;
    case APP_CHALLENGE_Q4:
        if (count != NULL) {
            *count = (uint8_t)(sizeof(k_q4_sequence) / sizeof(k_q4_sequence[0]));
        }
        return k_q4_sequence;
    case APP_CHALLENGE_NONE:
    default:
        break;
    }

    if (count != NULL) {
        *count = 0u;
    }
    return NULL;
}

static const phase_descriptor_t *get_phase_descriptor(app_challenge_t challenge, uint8_t sequence_index)
{
    uint8_t count = 0u;
    const phase_descriptor_t *sequence = get_sequence(challenge, &count);

    if ((sequence == NULL) || (sequence_index >= count)) {
        return NULL;
    }
    return &sequence[sequence_index];
}

static uint8_t resolve_next_sequence(const main_task_ctx_t *ctx,
                                     uint8_t *next_sequence_index,
                                     uint8_t *next_lap_index)
{
    uint8_t count = 0u;

    (void)get_sequence(ctx->challenge_active, &count);
    if ((count == 0u) || (ctx->sequence_index >= count)) {
        return 0u;
    }

    *next_sequence_index = (uint8_t)(ctx->sequence_index + 1u);
    *next_lap_index = ctx->lap_index;

    if (ctx->challenge_active == APP_CHALLENGE_Q4) {
        if (ctx->sequence_index == 4u) {
            if (ctx->lap_index < ctx->lap_total) {
                *next_sequence_index = 1u;
                *next_lap_index = (uint8_t)(ctx->lap_index + 1u);
                return 1u;
            }
            *next_sequence_index = 5u;
            return 1u;
        }
    }

    if (*next_sequence_index >= count) {
        return 0u;
    }
    return 1u;
}

static uint8_t resolve_next_phase_descriptor(const main_task_ctx_t *ctx,
                                             uint8_t *next_sequence_index,
                                             uint8_t *next_lap_index,
                                             const phase_descriptor_t **next_phase)
{
    const phase_descriptor_t *phase;

    if (next_phase != NULL) {
        *next_phase = NULL;
    }

    if ((ctx == NULL) ||
        (next_sequence_index == NULL) ||
        (next_lap_index == NULL) ||
        (next_phase == NULL)) {
        return 0u;
    }

    if (resolve_next_sequence(ctx, next_sequence_index, next_lap_index) == 0u) {
        return 0u;
    }

    phase = get_phase_descriptor(ctx->challenge_active, *next_sequence_index);
    if (phase == NULL) {
        return 0u;
    }

    *next_phase = phase;
    return 1u;
}

static float get_phase_distance_m(const main_task_ctx_t *ctx,
                                  const chassis_feedback_t *feedback)
{
    float left_delta;
    float right_delta;

    left_delta = absf((float)(feedback->left_count - ctx->phase_start_left_count));
    right_delta = absf((float)(feedback->right_count - ctx->phase_start_right_count));
    return ((left_delta + right_delta) * 0.5f) * MAIN_M_PER_COUNT;
}

static float apply_speed_slew(float current_v_mps, float target_v_mps)
{
    float delta = target_v_mps - current_v_mps;
    float limit = ((delta >= 0.0f) ? MAIN_V_ACCEL_MPS2 : MAIN_V_DECEL_MPS2) * MAIN_DT_S;

    if (delta > limit) {
        delta = limit;
    } else if (delta < -limit) {
        delta = -limit;
    }

    return current_v_mps + delta;
}

static float select_arc_speed_limit(float base_speed_mps, float line_error)
{
    float abs_error = absf(line_error);

    if (abs_error >= (float)MAIN_ARC_MAX_ERROR_THRESHOLD) {
        return clampf(base_speed_mps, 0.0f, 0.24f);
    }
    if (abs_error >= (float)MAIN_ARC_MEDIUM_ERROR_THRESHOLD) {
        return clampf(base_speed_mps, 0.0f, 0.34f);
    }
    if (abs_error >= (float)MAIN_ARC_FAST_ERROR_THRESHOLD) {
        return clampf(base_speed_mps, 0.0f, 0.44f);
    }
    return base_speed_mps;
}

static float select_edge_boost(const arc_profile_t *profile, float line_error)
{
    float abs_error = absf(line_error);
    float max_scale;

    if (profile == NULL) {
        return 1.0f;
    }

    max_scale = profile->edge_boost_max_scale;

    if (abs_error <= (float)MAIN_ARC_EDGE_BOOST_THRESHOLD) {
        return 1.0f;
    }
    if (abs_error >= (float)MAIN_ARC_MAX_ERROR_THRESHOLD) {
        return max_scale;
    }

    return lerpf(1.0f,
                 max_scale,
                 (abs_error - (float)MAIN_ARC_EDGE_BOOST_THRESHOLD) /
                     (float)(MAIN_ARC_MAX_ERROR_THRESHOLD - MAIN_ARC_EDGE_BOOST_THRESHOLD));
}

static float select_gap_speed(const phase_descriptor_t *phase, float phase_distance_m)
{
    float remaining_m = phase->nominal_distance_m - phase_distance_m;

    if (remaining_m <= 0.18f) {
        return MAIN_GAP_END_SPEED_MPS;
    }
    if (remaining_m < 0.40f) {
        return lerpf(MAIN_GAP_END_SPEED_MPS,
                     MAIN_GAP_CRUISE_SPEED_MPS,
                     clampf((remaining_m - 0.18f) / 0.22f, 0.0f, 1.0f));
    }
    return MAIN_GAP_CRUISE_SPEED_MPS;
}

static uint8_t phase_requires_centered_d_exit(const phase_descriptor_t *phase)
{
    return ((phase != NULL) &&
            (phase->action == APP_PHASE_ACTION_GAP_TRAVERSE) &&
            (phase->complete_event == APP_EVENT_PASS_D))
               ? 1u
               : 0u;
}

static uint8_t phase_is_arc_track(const phase_descriptor_t *phase)
{
    return ((phase != NULL) &&
            (phase->action == APP_PHASE_ACTION_ARC_TRACK) &&
            (phase->arc_profile != NULL))
               ? 1u
               : 0u;
}

static uint8_t phase_is_final_a_arc(const phase_descriptor_t *phase,
                                    const phase_descriptor_t *next_phase)
{
    return ((phase != NULL) &&
            (next_phase != NULL) &&
            (phase->phase == APP_CHALLENGE_PHASE_ARC_DA) &&
            (phase->complete_event == APP_EVENT_PASS_A) &&
            (next_phase->action == APP_PHASE_ACTION_STOP_AND_SIGNAL) &&
            (next_phase->phase == APP_CHALLENGE_PHASE_STOP_A))
               ? 1u
               : 0u;
}

static float select_gap_arc_fallback_margin_m(const phase_descriptor_t *phase,
                                              const phase_descriptor_t *next_phase)
{
    if ((phase != NULL) &&
        (next_phase != NULL) &&
        (phase->phase == APP_CHALLENGE_PHASE_GAP_BD) &&
        (next_phase->phase == APP_CHALLENGE_PHASE_ARC_DA)) {
        return MAIN_GAP_BD_ARC_FALLBACK_MARGIN_M;
    }
    return MAIN_GAP_ARC_FALLBACK_MARGIN_M;
}

static uint8_t gap_arc_encoder_fallback_ready(const phase_descriptor_t *phase,
                                              const phase_descriptor_t *next_phase,
                                              float phase_distance_m)
{
    if ((phase == NULL) || (phase_is_arc_track(next_phase) == 0u)) {
        return 0u;
    }

    return (phase_distance_m >=
            (phase->nominal_distance_m +
             select_gap_arc_fallback_margin_m(phase, next_phase)))
               ? 1u
               : 0u;
}

static uint8_t gap_arc_encoder_fallback_window(const phase_descriptor_t *phase,
                                               const phase_descriptor_t *next_phase,
                                               float phase_distance_m)
{
    if ((phase == NULL) || (phase_is_arc_track(next_phase) == 0u)) {
        return 0u;
    }

    return (phase_distance_m >= phase->nominal_distance_m) ? 1u : 0u;
}

static uint16_t select_gap_d_confirm_ms(const phase_descriptor_t *phase)
{
    if ((phase != NULL) && (phase->phase == APP_CHALLENGE_PHASE_GAP_BD)) {
        return MAIN_GAP_BD_D_CONFIRM_MS;
    }
    return MAIN_GAP_D_CENTER_CONFIRM_MS;
}

static int16_t select_gap_d_pos_max(const phase_descriptor_t *phase)
{
    if ((phase != NULL) && (phase->phase == APP_CHALLENGE_PHASE_GAP_BD)) {
        return MAIN_GAP_BD_D_POS_MAX;
    }
    return MAIN_GAP_D_CENTER_POS_MAX;
}

static uint8_t feedback_is_centered_for_d_exit(const phase_descriptor_t *phase,
                                               const chassis_feedback_t *feedback)
{
    return ((feedback != NULL) &&
            (feedback->line_detected != 0u) &&
            (absi16(feedback->line_position) <= select_gap_d_pos_max(phase)))
               ? 1u
               : 0u;
}

static void apply_gap_d_exit_centering(const phase_descriptor_t *phase,
                                       line_controller_t *line_controller,
                                       const chassis_feedback_t *feedback,
                                       chassis_command_t *command)
{
    float control_w;

    if ((phase_requires_centered_d_exit(phase) == 0u) ||
        (feedback == NULL) ||
        (line_controller == NULL) ||
        (command == NULL)) {
        return;
    }

    if (feedback->line_detected == 0u) {
        LineController_Reset(line_controller);
        return;
    }

    control_w = LineController_Update(line_controller,
                                      (float)feedback->line_position,
                                      feedback->line_bits,
                                      feedback->line_detected,
                                      MAIN_DT_S);
    control_w = apply_line_steer_sign(control_w * MAIN_GAP_D_CENTER_W_SCALE);
    control_w = clampf(control_w,
                       -MAIN_GAP_D_CENTER_W_LIMIT_RADPS,
                       MAIN_GAP_D_CENTER_W_LIMIT_RADPS);
    command->w_radps = clampf(command->w_radps + control_w,
                              -MAIN_GAP_W_LIMIT_RADPS,
                              MAIN_GAP_W_LIMIT_RADPS);
}

static float select_basic_straight_speed(float target_speed_mps, float remaining_m)
{
    target_speed_mps = clampf(target_speed_mps,
                              MAIN_BASIC_MIN_SPEED_MPS,
                              MAIN_BASIC_MAX_SPEED_MPS);

    if (remaining_m <= 0.0f) {
        return 0.0f;
    }
    if (remaining_m <= 0.10f) {
        return clampf(target_speed_mps,
                      MAIN_BASIC_MIN_SPEED_MPS,
                      MAIN_BASIC_STRAIGHT_END_SPEED_MPS);
    }
    if (remaining_m < 0.30f) {
        return lerpf(MAIN_BASIC_STRAIGHT_END_SPEED_MPS,
                     target_speed_mps,
                     clampf((remaining_m - 0.10f) / 0.20f, 0.0f, 1.0f));
    }
    return target_speed_mps;
}

static float select_arc_zone_speed(const arc_profile_t *profile, float phase_distance_m)
{
    if (phase_distance_m < profile->front_end_m) {
        return profile->speed_front_mps;
    }
    if (phase_distance_m < profile->rear_start_m) {
        return profile->speed_mid_mps;
    }
    return profile->speed_rear_mps;
}

static float select_arc_entry_start_speed(const phase_descriptor_t *phase)
{
    if ((phase != NULL) && (phase->phase == APP_CHALLENGE_PHASE_ARC_DA)) {
        return MAIN_ARC_ENTRY_DA_START_SPEED_MPS;
    }
    return MAIN_ARC_ENTRY_START_SPEED_MPS;
}

static float apply_arc_entry_speed_limit(const phase_descriptor_t *phase,
                                         float zone_speed_mps,
                                         float phase_distance_m)
{
    float entry_speed_mps;
    float t;

    if (phase_distance_m >= MAIN_ARC_ENTRY_RAMP_END_M) {
        return zone_speed_mps;
    }

    entry_speed_mps = select_arc_entry_start_speed(phase);
    if (phase_distance_m <= MAIN_ARC_ENTRY_HOLD_DISTANCE_M) {
        return clampf(zone_speed_mps, 0.0f, entry_speed_mps);
    }

    t = (phase_distance_m - MAIN_ARC_ENTRY_HOLD_DISTANCE_M) /
        (MAIN_ARC_ENTRY_RAMP_END_M - MAIN_ARC_ENTRY_HOLD_DISTANCE_M);
    return clampf(zone_speed_mps,
                  0.0f,
                  lerpf(entry_speed_mps, zone_speed_mps, clampf(t, 0.0f, 1.0f)));
}

static float select_arc_turn_scale(const arc_profile_t *profile, float phase_distance_m)
{
    if (phase_distance_m < profile->front_end_m) {
        return profile->turn_scale_front;
    }
    if (phase_distance_m < profile->rear_start_m) {
        return profile->turn_scale_mid;
    }
    return profile->turn_scale_rear;
}

static float apply_arc_entry_w_limit(const arc_profile_t *profile,
                                     float w_radps,
                                     float phase_distance_m)
{
    float limit_radps;
    float profile_limit_radps;
    float t;

    profile_limit_radps = (profile != NULL) ? profile->w_limit_radps : MAIN_ARC_W_LIMIT_RADPS;
    profile_limit_radps = clampf(profile_limit_radps,
                                 MAIN_ARC_ENTRY_W_LIMIT_RADPS,
                                 MAIN_ARC_W_LIMIT_RADPS);

    if (phase_distance_m >= MAIN_ARC_ENTRY_RAMP_END_M) {
        return clampf(w_radps, -profile_limit_radps, profile_limit_radps);
    }

    if (phase_distance_m <= MAIN_ARC_ENTRY_HOLD_DISTANCE_M) {
        limit_radps = MAIN_ARC_ENTRY_W_LIMIT_RADPS;
    } else {
        t = (phase_distance_m - MAIN_ARC_ENTRY_HOLD_DISTANCE_M) /
            (MAIN_ARC_ENTRY_RAMP_END_M - MAIN_ARC_ENTRY_HOLD_DISTANCE_M);
        limit_radps = lerpf(MAIN_ARC_ENTRY_W_LIMIT_RADPS,
                            profile_limit_radps,
                            clampf(t, 0.0f, 1.0f));
    }

    return clampf(w_radps, -limit_radps, limit_radps);
}

static float apply_final_a_speed_limit(const phase_descriptor_t *phase,
                                       const phase_descriptor_t *next_phase,
                                       float target_speed_mps,
                                       float phase_distance_m)
{
    if ((phase_is_final_a_arc(phase, next_phase) == 0u) ||
        (phase_distance_m < MAIN_FINAL_A_ALIGN_START_M)) {
        return target_speed_mps;
    }

    return clampf(target_speed_mps, 0.0f, MAIN_FINAL_A_ALIGN_SPEED_MPS);
}

static uint8_t phase_is_q4_a_lap_exit(const main_task_ctx_t *ctx,
                                      const phase_descriptor_t *next_phase)
{
    return ((ctx != NULL) &&
            (next_phase != NULL) &&
            (ctx->challenge_active == APP_CHALLENGE_Q4) &&
            (ctx->sequence_index == 4u) &&
            (ctx->lap_index < ctx->lap_total) &&
            (next_phase->action == APP_PHASE_ACTION_GAP_TRAVERSE) &&
            (next_phase->phase == APP_CHALLENGE_PHASE_GAP_AC))
               ? 1u
               : 0u;
}

static float get_arc_yaw_delta_deg(const main_task_ctx_t *ctx,
                                   const chassis_feedback_t *feedback)
{
    if ((ctx == NULL) || (feedback == NULL)) {
        return 0.0f;
    }

    return absf(wrap_angle_deg(feedback->yaw_deg - ctx->arc_entry_yaw_deg));
}

static uint8_t final_a_exit_gate_ready(const main_task_ctx_t *ctx,
                                       const phase_descriptor_t *phase,
                                       const chassis_feedback_t *feedback)
{
    float yaw_delta_deg;

    if ((ctx == NULL) || (phase == NULL) ||
        (phase->phase != APP_CHALLENGE_PHASE_ARC_DA) ||
        (phase->arc_profile == NULL)) {
        return 0u;
    }

    if ((feedback == NULL) ||
        (feedback->line_detected != 0u) ||
        (ctx->line_missing_ms < phase->arc_profile->lost_confirm_ms)) {
        return 0u;
    }

    if (ctx->phase_distance_m < MAIN_FINAL_A_EXIT_MIN_DISTANCE_M) {
        return 0u;
    }

    yaw_delta_deg = get_arc_yaw_delta_deg(ctx, feedback);
    if (yaw_delta_deg >= MAIN_FINAL_A_EXIT_YAW_MIN_DEG) {
        return 1u;
    }

    return (ctx->phase_distance_m >= MAIN_FINAL_A_EXIT_MAX_DISTANCE_M) ? 1u : 0u;
}

static float get_arc_inner_bias_w(const arc_profile_t *profile, float phase_distance_m)
{
    float t;

    if ((profile == NULL) || (phase_distance_m <= profile->inner_bias_start_m)) {
        return 0.0f;
    }

    t = (phase_distance_m - profile->inner_bias_start_m) /
        (profile->inner_bias_full_m - profile->inner_bias_start_m);
    t = clampf(t, 0.0f, 1.0f);
    return (float)profile->inner_bias_sign * profile->inner_bias_w_radps * t;
}

static float apply_line_steer_sign(float w_radps)
{
    return MAIN_LINE_STEER_SIGN * w_radps;
}

static float apply_heading_steer_sign(float w_radps)
{
    return MAIN_HEADING_STEER_SIGN * w_radps;
}

static float geometry_to_imu_heading_deg(float geometry_heading_deg)
{
    return wrap_angle_deg(MAIN_FIELD_TO_IMU_YAW_SIGN * geometry_heading_deg);
}

static float get_field_heading_deg(const main_task_ctx_t *ctx,
                                   const phase_descriptor_t *phase)
{
    if ((ctx == NULL) || (phase == NULL)) {
        return 0.0f;
    }
    return wrap_angle_deg(ctx->field_yaw_offset_deg +
                          geometry_to_imu_heading_deg(phase->geometry_heading_deg));
}

static float get_phase_target_distance_m(const phase_descriptor_t *phase)
{
    if (phase == NULL) {
        return 0.0f;
    }
    if (phase->action == APP_PHASE_ACTION_GAP_TRAVERSE) {
        return phase->nominal_distance_m;
    }
    if ((phase->action == APP_PHASE_ACTION_ARC_TRACK) &&
        (phase->arc_profile != NULL)) {
        return phase->arc_profile->expected_length_m;
    }
    return 0.0f;
}

static void publish_runtime_state(const main_task_ctx_t *ctx,
                                  app_challenge_info_t *challenge,
                                  const phase_descriptor_t *phase,
                                  app_challenge_status_t status)
{
    app_state_get_challenge(challenge);
    challenge->active = ctx->challenge_active;
    challenge->status = status;
    challenge->phase = (phase != NULL) ? phase->phase : APP_CHALLENGE_PHASE_IDLE;
    challenge->action = (phase != NULL) ? phase->action : APP_PHASE_ACTION_NONE;
    challenge->checkpoint_count = ctx->checkpoint_count;
    challenge->lap_index = ctx->lap_index;
    challenge->lap_total = ctx->lap_total;
    challenge->geometry_heading_deg = ctx->geometry_heading_deg;
    challenge->hold_heading_deg = ctx->hold_heading_deg;
    challenge->heading_error_deg = ctx->heading_error_deg;
    challenge->phase_distance_m = ctx->phase_distance_m;
    challenge->target_distance_m = get_phase_target_distance_m(phase);
    challenge->target_speed_mps = ctx->target_speed_mps;
    app_state_set_challenge(challenge);
    app_state_set_main_state(ctx->state);
}

static void publish_basic_runtime_state(const main_task_ctx_t *ctx,
                                        app_phase_action_t action,
                                        app_challenge_phase_t phase,
                                        app_challenge_status_t status)
{
    app_challenge_info_t challenge;

    app_state_get_challenge(&challenge);
    challenge.active = APP_CHALLENGE_NONE;
    challenge.status = status;
    challenge.phase = phase;
    challenge.action = action;
    challenge.checkpoint_count = 0u;
    challenge.lap_index = 1u;
    challenge.lap_total = 1u;
    challenge.geometry_heading_deg = ctx->geometry_heading_deg;
    challenge.hold_heading_deg = ctx->hold_heading_deg;
    challenge.heading_error_deg = ctx->heading_error_deg;
    challenge.phase_distance_m = ctx->phase_distance_m;
    challenge.target_distance_m = ctx->target_distance_m;
    challenge.target_speed_mps = ctx->target_speed_mps;
    app_state_set_challenge(&challenge);
    app_state_set_main_state(ctx->state);
}

static void reset_main_context(main_task_ctx_t *ctx,
                               line_controller_t *line_controller,
                               yaw_controller_t *yaw_controller)
{
    if (ctx == NULL) {
        return;
    }

    memset(ctx, 0, sizeof(*ctx));
    ctx->active_mode = APP_MODE_STOP;
    ctx->challenge_active = APP_CHALLENGE_NONE;
    ctx->lap_index = 1u;
    ctx->state = APP_MAIN_STATE_IDLE;
    LineController_Reset(line_controller);
    YawController_Reset(yaw_controller);
    app_state_set_main_state(APP_MAIN_STATE_IDLE);
}

static void enter_phase(main_task_ctx_t *ctx,
                        const phase_descriptor_t *phase,
                        const chassis_feedback_t *feedback,
                        line_controller_t *line_controller,
                        yaw_controller_t *yaw_controller)
{
    ctx->align_stable_ms = 0u;
    ctx->line_seen_ms = 0u;
    ctx->line_missing_ms = 0u;
    ctx->gap_left_line = (feedback->line_detected != 0u) ? 0u : 1u;
    ctx->arc_has_seen_line = (feedback->line_detected != 0u) ? 1u : 0u;
    ctx->phase_start_left_count = feedback->left_count;
    ctx->phase_start_right_count = feedback->right_count;
    ctx->arc_last_error = (float)feedback->line_position;
    ctx->phase_distance_m = 0.0f;
    ctx->target_distance_m = get_phase_target_distance_m(phase);
    ctx->arc_entry_yaw_deg = feedback->yaw_deg;
    ctx->final_a_comp_start_distance_m = 0.0f;
    ctx->final_a_compensating = 0u;
    ctx->geometry_heading_deg = phase->geometry_heading_deg;
    if (phase->action == APP_PHASE_ACTION_GAP_TRAVERSE) {
        ctx->hold_heading_deg = get_field_heading_deg(ctx, phase);
    } else {
        ctx->hold_heading_deg = feedback->yaw_deg;
    }
    ctx->heading_error_deg = 0.0f;
    ctx->target_speed_mps = ctx->current_v_mps;

    LineController_Reset(line_controller);
    YawController_Reset(yaw_controller);

    switch (phase->action) {
    case APP_PHASE_ACTION_ALIGN_START:
        ctx->current_v_mps = 0.0f;
        ctx->state = APP_MAIN_STATE_ALIGN;
        break;
    case APP_PHASE_ACTION_GAP_TRAVERSE:
        ctx->state = APP_MAIN_STATE_GAP;
        break;
    case APP_PHASE_ACTION_ARC_TRACK:
        if (feedback->line_detected != 0u) {
            ctx->state = APP_MAIN_STATE_ARC_TRACK;
        } else if (feedback->line_position <= 0) {
            ctx->state = APP_MAIN_STATE_ARC_LOST_LEFT;
        } else {
            ctx->state = APP_MAIN_STATE_ARC_LOST_RIGHT;
        }
        break;
    case APP_PHASE_ACTION_STOP_AND_SIGNAL:
        ctx->current_v_mps = 0.0f;
        ctx->state = APP_MAIN_STATE_STOPPED;
        break;
    case APP_PHASE_ACTION_NONE:
    default:
        ctx->state = APP_MAIN_STATE_IDLE;
        break;
    }
}

static void finish_challenge(main_task_ctx_t *ctx,
                             line_controller_t *line_controller,
                             yaw_controller_t *yaw_controller,
                             app_challenge_info_t *challenge,
                             const phase_descriptor_t *stop_phase)
{
    chassis_command_t command = {0};

    ctx->current_v_mps = 0.0f;
    ctx->target_speed_mps = 0.0f;
    ctx->phase_distance_m = 0.0f;
    ctx->target_distance_m = 0.0f;
    ctx->heading_error_deg = 0.0f;
    ctx->hold_heading_deg = 0.0f;
    ctx->geometry_heading_deg = 0.0f;
    ctx->state = APP_MAIN_STATE_STOPPED;

    app_state_emit_event(APP_EVENT_STOP);
    app_state_get_challenge(challenge);
    challenge->active = APP_CHALLENGE_NONE;
    challenge->status = APP_CHALLENGE_STATUS_DONE;
    challenge->phase = (stop_phase != NULL) ? stop_phase->phase : APP_CHALLENGE_PHASE_IDLE;
    challenge->action = APP_PHASE_ACTION_STOP_AND_SIGNAL;
    challenge->checkpoint_count = ctx->checkpoint_count;
    challenge->lap_index = ctx->lap_index;
    challenge->lap_total = ctx->lap_total;
    challenge->geometry_heading_deg = 0.0f;
    challenge->hold_heading_deg = 0.0f;
    challenge->heading_error_deg = 0.0f;
    challenge->phase_distance_m = 0.0f;
    challenge->target_distance_m = 0.0f;
    challenge->target_speed_mps = 0.0f;
    app_state_set_challenge(challenge);
    app_state_set_main_state(APP_MAIN_STATE_STOPPED);

    command.stop = 1u;
    command.enable_closed_loop = 0u;
    app_state_set_command(&command);
    app_state_set_mode(APP_MODE_STOP);

    reset_main_context(ctx, line_controller, yaw_controller);
}

static float run_gap_traverse(main_task_ctx_t *ctx,
                              const phase_descriptor_t *phase,
                              yaw_controller_t *yaw_controller,
                              const chassis_feedback_t *feedback,
                              chassis_command_t *command)
{
    float heading_term = 0.0f;

    if ((feedback->imu_ready != 0u) && (feedback->imu_stable != 0u)) {
        ctx->heading_error_deg = wrap_angle_deg(ctx->hold_heading_deg - feedback->yaw_deg);
        heading_term = YawController_Update(yaw_controller,
                                            ctx->hold_heading_deg,
                                            feedback->yaw_deg,
                                            feedback->gyro_z,
                                            MAIN_DT_S);
    } else {
        ctx->heading_error_deg = 0.0f;
        YawController_Reset(yaw_controller);
    }

    command->w_radps = clampf(apply_heading_steer_sign(heading_term),
                              -MAIN_GAP_W_LIMIT_RADPS,
                              MAIN_GAP_W_LIMIT_RADPS);
    ctx->state = APP_MAIN_STATE_GAP;
    return select_gap_speed(phase, ctx->phase_distance_m);
}

static float run_arc_track(main_task_ctx_t *ctx,
                           const phase_descriptor_t *phase,
                           line_controller_t *line_controller,
                           const chassis_feedback_t *feedback,
                           chassis_command_t *command)
{
    float zone_speed_mps;

    zone_speed_mps = select_arc_zone_speed(phase->arc_profile, ctx->phase_distance_m);
    zone_speed_mps = apply_arc_entry_speed_limit(phase,
                                                 zone_speed_mps,
                                                 ctx->phase_distance_m);

    if (feedback->line_detected != 0u) {
        float line_error = (float)feedback->line_position;
        float turn_scale = select_arc_turn_scale(phase->arc_profile, ctx->phase_distance_m);
        float control_w;

        ctx->arc_has_seen_line = 1u;
        ctx->arc_last_error = line_error;
        ctx->line_missing_ms = 0u;
        ctx->heading_error_deg = 0.0f;
        ctx->state = APP_MAIN_STATE_ARC_TRACK;

        control_w = LineController_Update(line_controller,
                                          line_error,
                                          feedback->line_bits,
                                          feedback->line_detected,
                                          MAIN_DT_S);
        control_w *= select_edge_boost(phase->arc_profile, line_error) * turn_scale;
        control_w += get_arc_inner_bias_w(phase->arc_profile, ctx->phase_distance_m);
        control_w = apply_line_steer_sign(control_w);
        command->w_radps = apply_arc_entry_w_limit(phase->arc_profile,
                                                   control_w,
                                                   ctx->phase_distance_m);
        return select_arc_speed_limit(zone_speed_mps, line_error);
    }

    LineController_Reset(line_controller);
    if ((ctx->arc_has_seen_line == 0u) || (ctx->arc_last_error <= 0.0f)) {
        command->w_radps = apply_arc_entry_w_limit(
            phase->arc_profile,
            apply_line_steer_sign(MAIN_ARC_SEARCH_W_RADPS),
            ctx->phase_distance_m);
        ctx->state = APP_MAIN_STATE_ARC_LOST_LEFT;
    } else {
        command->w_radps = apply_arc_entry_w_limit(
            phase->arc_profile,
            apply_line_steer_sign(-MAIN_ARC_SEARCH_W_RADPS),
            ctx->phase_distance_m);
        ctx->state = APP_MAIN_STATE_ARC_LOST_RIGHT;
    }

    if (ctx->line_missing_ms < phase->arc_profile->lost_confirm_ms) {
        return clampf(ctx->current_v_mps,
                      MAIN_ARC_SEARCH_SPEED_MPS,
                      zone_speed_mps);
    }
    return MAIN_ARC_SEARCH_SPEED_MPS;
}

static uint8_t run_final_a_heading_align(main_task_ctx_t *ctx,
                                         const phase_descriptor_t *phase,
                                         yaw_controller_t *yaw_controller,
                                         const chassis_feedback_t *feedback,
                                         chassis_command_t *command)
{
    float target_heading_deg;
    float heading_term;

    if ((ctx == NULL) || (phase == NULL) || (feedback == NULL) ||
        (command == NULL)) {
        return 1u;
    }

    target_heading_deg = get_field_heading_deg(ctx, phase);
    ctx->hold_heading_deg = target_heading_deg;
    ctx->geometry_heading_deg = phase->geometry_heading_deg;

    if ((feedback->imu_ready == 0u) || (feedback->imu_stable == 0u)) {
        ctx->heading_error_deg = 0.0f;
        YawController_Reset(yaw_controller);
        return 1u;
    }

    ctx->heading_error_deg = wrap_angle_deg(target_heading_deg - feedback->yaw_deg);
    heading_term = YawController_Update(yaw_controller,
                                        target_heading_deg,
                                        feedback->yaw_deg,
                                        feedback->gyro_z,
                                        MAIN_DT_S);
    command->w_radps = clampf(apply_heading_steer_sign(heading_term),
                              -MAIN_FINAL_A_ALIGN_W_LIMIT_RADPS,
                              MAIN_FINAL_A_ALIGN_W_LIMIT_RADPS);
    ctx->target_speed_mps = MAIN_FINAL_A_ALIGN_SPEED_MPS;
    ctx->state = APP_MAIN_STATE_ARC_EXIT_ALIGN;
    return 0u;
}

static uint8_t run_final_a_compensate(main_task_ctx_t *ctx,
                                      const phase_descriptor_t *phase,
                                      yaw_controller_t *yaw_controller,
                                      const chassis_feedback_t *feedback,
                                      chassis_command_t *command)
{
    if ((ctx == NULL) || (phase == NULL) || (feedback == NULL) ||
        (command == NULL)) {
        return 1u;
    }

    if (ctx->final_a_compensating == 0u) {
        if (final_a_exit_gate_ready(ctx, phase, feedback) == 0u) {
            if (run_final_a_heading_align(ctx,
                                          phase,
                                          yaw_controller,
                                          feedback,
                                          command) != 0u) {
                command->w_radps = 0.0f;
                ctx->target_speed_mps = MAIN_FINAL_A_ALIGN_SPEED_MPS;
                ctx->state = APP_MAIN_STATE_ARC_EXIT_ALIGN;
            }
            return 0u;
        }
        ctx->final_a_compensating = 1u;
        ctx->final_a_comp_start_distance_m = ctx->phase_distance_m;
        YawController_Reset(yaw_controller);
    }

    if ((ctx->phase_distance_m - ctx->final_a_comp_start_distance_m) >=
        MAIN_FINAL_A_COMPENSATE_M) {
        return 1u;
    }

    (void)run_final_a_heading_align(ctx, phase, yaw_controller, feedback, command);
    ctx->target_speed_mps = MAIN_FINAL_A_ALIGN_SPEED_MPS;
    ctx->state = APP_MAIN_STATE_ARC_EXIT_ALIGN;
    return 0u;
}

static uint8_t run_arc_exit_heading_align(main_task_ctx_t *ctx,
                                          const phase_descriptor_t *next_phase,
                                          yaw_controller_t *yaw_controller,
                                          const chassis_feedback_t *feedback,
                                          chassis_command_t *command)
{
    float target_heading_deg;
    float heading_term;
    float align_speed_mps = MAIN_ARC_EXIT_ALIGN_SPEED_MPS;
    float w_limit_radps = MAIN_ARC_EXIT_ALIGN_W_LIMIT_RADPS;
    float abs_error;

    if ((ctx == NULL) || (next_phase == NULL) || (feedback == NULL) ||
        (command == NULL) ||
        (next_phase->action != APP_PHASE_ACTION_GAP_TRAVERSE)) {
        return 1u;
    }

    target_heading_deg = get_field_heading_deg(ctx, next_phase);
    ctx->hold_heading_deg = target_heading_deg;
    ctx->geometry_heading_deg = next_phase->geometry_heading_deg;

    if ((ctx->challenge_active == APP_CHALLENGE_Q3 ||
         ctx->challenge_active == APP_CHALLENGE_Q4) &&
        (ctx->sequence_index == 2u) &&
        (next_phase->phase == APP_CHALLENGE_PHASE_GAP_BD)) {
        align_speed_mps = MAIN_ARC_CB_EXIT_ALIGN_SPEED_MPS;
        w_limit_radps = MAIN_ARC_CB_EXIT_ALIGN_W_LIMIT_RADPS;
    } else if (phase_is_q4_a_lap_exit(ctx, next_phase) != 0u) {
        align_speed_mps = MAIN_ARC_Q4_A_EXIT_ALIGN_SPEED_MPS;
        w_limit_radps = MAIN_ARC_Q4_A_EXIT_ALIGN_W_LIMIT_RADPS;
    }

    if ((feedback->imu_ready == 0u) || (feedback->imu_stable == 0u)) {
        ctx->heading_error_deg = 0.0f;
        YawController_Reset(yaw_controller);
        return 1u;
    }

    ctx->heading_error_deg = wrap_angle_deg(target_heading_deg - feedback->yaw_deg);
    abs_error = absf(ctx->heading_error_deg);

    if ((abs_error <= MAIN_ARC_EXIT_HEADING_TOL_DEG) ||
        ((ctx->phase_distance_m >=
            (ctx->target_distance_m + MAIN_ARC_EXIT_MAX_OVERRUN_M)) &&
         (abs_error <= MAIN_ARC_EXIT_OVERRUN_TOL_DEG))) {
        return 1u;
    }

    heading_term = YawController_Update(yaw_controller,
                                        target_heading_deg,
                                        feedback->yaw_deg,
                                        feedback->gyro_z,
                                        MAIN_DT_S);
    command->w_radps = clampf(apply_heading_steer_sign(heading_term),
                              -w_limit_radps,
                              w_limit_radps);
    ctx->target_speed_mps = align_speed_mps;
    ctx->state = APP_MAIN_STATE_ARC_EXIT_ALIGN;
    return 0u;
}

static void enter_basic_mode(main_task_ctx_t *ctx,
                             app_mode_t mode,
                             const chassis_command_t *requested_command,
                             const chassis_feedback_t *feedback,
                             line_controller_t *line_controller,
                             yaw_controller_t *yaw_controller)
{
    float requested_speed;
    float requested_distance;

    reset_main_context(ctx, line_controller, yaw_controller);
    ctx->active = 1u;
    ctx->active_mode = mode;
    ctx->challenge_active = APP_CHALLENGE_NONE;
    ctx->phase_start_left_count = feedback->left_count;
    ctx->phase_start_right_count = feedback->right_count;
    ctx->phase_distance_m = 0.0f;
    ctx->hold_heading_deg = feedback->yaw_deg;
    ctx->geometry_heading_deg = feedback->yaw_deg;
    ctx->heading_error_deg = 0.0f;
    ctx->arc_last_error = (float)feedback->line_position;
    ctx->arc_has_seen_line = (feedback->line_detected != 0u) ? 1u : 0u;

    requested_speed = (requested_command != NULL) ? requested_command->v_mps : 0.0f;
    requested_distance = (requested_command != NULL) ? requested_command->left_speed_mps : 0.0f;

    if (mode == APP_MODE_STRAIGHT_TEST) {
        if (requested_distance <= 0.0f) {
            requested_distance = MAIN_BASIC_STRAIGHT_DEFAULT_DISTANCE_M;
        }
        if (requested_speed <= 0.0f) {
            requested_speed = MAIN_BASIC_STRAIGHT_DEFAULT_SPEED_MPS;
        }
        ctx->target_distance_m = clampf(requested_distance,
                                        MAIN_BASIC_MIN_DISTANCE_M,
                                        MAIN_BASIC_MAX_DISTANCE_M);
        ctx->cruise_speed_mps = clampf(requested_speed,
                                       MAIN_BASIC_MIN_SPEED_MPS,
                                       MAIN_BASIC_MAX_SPEED_MPS);
        ctx->target_speed_mps = 0.0f;
        ctx->state = APP_MAIN_STATE_GAP;
        publish_basic_runtime_state(ctx,
                                    APP_PHASE_ACTION_GAP_TRAVERSE,
                                    APP_CHALLENGE_PHASE_GAP_AB,
                                    APP_CHALLENGE_STATUS_RUNNING);
    } else {
        if (requested_speed <= 0.0f) {
            requested_speed = MAIN_BASIC_LINE_DEFAULT_SPEED_MPS;
        }
        ctx->target_distance_m = clampf(requested_distance,
                                        0.0f,
                                        MAIN_BASIC_MAX_DISTANCE_M);
        ctx->cruise_speed_mps = clampf(requested_speed,
                                       MAIN_BASIC_MIN_SPEED_MPS,
                                       MAIN_BASIC_MAX_SPEED_MPS);
        ctx->target_speed_mps = ctx->cruise_speed_mps;
        ctx->state = (feedback->line_detected != 0u)
                         ? APP_MAIN_STATE_ARC_TRACK
                         : APP_MAIN_STATE_ARC_LOST_LEFT;
        publish_basic_runtime_state(ctx,
                                    APP_PHASE_ACTION_ARC_TRACK,
                                    APP_CHALLENGE_PHASE_ARC_BC,
                                    APP_CHALLENGE_STATUS_RUNNING);
    }

    app_state_emit_event(APP_EVENT_START);
}

static float update_heading_hold(main_task_ctx_t *ctx,
                                 yaw_controller_t *yaw_controller,
                                 const chassis_feedback_t *feedback)
{
    if ((feedback->imu_ready != 0u) && (feedback->imu_stable != 0u)) {
        float pid_target;

        ctx->heading_error_deg = wrap_angle_deg(ctx->hold_heading_deg - feedback->yaw_deg);
        pid_target = feedback->yaw_deg + ctx->heading_error_deg;
        return YawController_Update(yaw_controller,
                                    pid_target,
                                    feedback->yaw_deg,
                                    feedback->gyro_z,
                                    MAIN_DT_S);
    }

    ctx->heading_error_deg = 0.0f;
    YawController_Reset(yaw_controller);
    return 0.0f;
}

static void stop_basic_mode(main_task_ctx_t *ctx,
                            line_controller_t *line_controller,
                            yaw_controller_t *yaw_controller)
{
    chassis_command_t command = {0};

    ctx->current_v_mps = 0.0f;
    ctx->target_speed_mps = 0.0f;
    ctx->state = APP_MAIN_STATE_STOPPED;
    publish_basic_runtime_state(ctx,
                                APP_PHASE_ACTION_STOP_AND_SIGNAL,
                                APP_CHALLENGE_PHASE_STOP_B,
                                APP_CHALLENGE_STATUS_DONE);
    app_state_emit_event(APP_EVENT_STOP);

    command.stop = 1u;
    command.enable_closed_loop = 0u;
    app_state_set_command(&command);
    app_state_set_mode(APP_MODE_STOP);
    reset_main_context(ctx, line_controller, yaw_controller);
}

static void run_basic_straight(main_task_ctx_t *ctx,
                               yaw_controller_t *yaw_controller,
                               const chassis_feedback_t *feedback,
                               chassis_command_t *command)
{
    float remaining_m;
    float heading_term;

    ctx->phase_distance_m = get_phase_distance_m(ctx, feedback);
    remaining_m = ctx->target_distance_m - ctx->phase_distance_m;
    heading_term = update_heading_hold(ctx, yaw_controller, feedback);

    command->stop = 0u;
    command->enable_closed_loop = 1u;
    command->w_radps = clampf(apply_heading_steer_sign(heading_term),
                              -MAIN_GAP_W_LIMIT_RADPS,
                              MAIN_GAP_W_LIMIT_RADPS);
    ctx->target_speed_mps = select_basic_straight_speed(ctx->cruise_speed_mps, remaining_m);
    ctx->current_v_mps = apply_speed_slew(ctx->current_v_mps, ctx->target_speed_mps);
    command->v_mps = ctx->current_v_mps;
    ctx->state = APP_MAIN_STATE_GAP;
}

static void run_basic_line(main_task_ctx_t *ctx,
                           line_controller_t *line_controller,
                           const chassis_feedback_t *feedback,
                           chassis_command_t *command)
{
    ctx->phase_distance_m = get_phase_distance_m(ctx, feedback);
    ctx->heading_error_deg = 0.0f;

    command->stop = 0u;
    command->enable_closed_loop = 1u;

    if (feedback->line_detected != 0u) {
        float line_error = (float)feedback->line_position;
        float control_w;
        float target_speed_mps;

        ctx->arc_has_seen_line = 1u;
        ctx->arc_last_error = line_error;
        ctx->line_missing_ms = 0u;
        ctx->state = APP_MAIN_STATE_ARC_TRACK;

        control_w = LineController_Update(line_controller,
                                          line_error,
                                          feedback->line_bits,
                                          feedback->line_detected,
                                          MAIN_DT_S);
        control_w = apply_line_steer_sign(control_w);
        command->w_radps = clampf(control_w,
                                  -MAIN_BASIC_LINE_W_LIMIT_RADPS,
                                  MAIN_BASIC_LINE_W_LIMIT_RADPS);
        target_speed_mps = select_arc_speed_limit(ctx->cruise_speed_mps, line_error);
        ctx->target_speed_mps = target_speed_mps;
        ctx->current_v_mps = apply_speed_slew(ctx->current_v_mps, target_speed_mps);
    } else {
        LineController_Reset(line_controller);
        ctx->line_missing_ms = (uint16_t)(ctx->line_missing_ms + MAIN_TASK_PERIOD_MS);
        if ((ctx->arc_has_seen_line == 0u) || (ctx->arc_last_error <= 0.0f)) {
            command->w_radps = apply_line_steer_sign(MAIN_ARC_SEARCH_W_RADPS);
            ctx->state = APP_MAIN_STATE_ARC_LOST_LEFT;
        } else {
            command->w_radps = apply_line_steer_sign(-MAIN_ARC_SEARCH_W_RADPS);
            ctx->state = APP_MAIN_STATE_ARC_LOST_RIGHT;
        }
        ctx->target_speed_mps = MAIN_BASIC_LINE_SEARCH_SPEED_MPS;
        ctx->current_v_mps = apply_speed_slew(ctx->current_v_mps,
                                              ctx->target_speed_mps);
    }

    command->v_mps = ctx->current_v_mps;
}

void main_task(void *arg)
{
    TickType_t next;
    main_task_ctx_t ctx;
    line_controller_t *line_controller;
    yaw_controller_t *yaw_controller;

    (void)arg;

    chassis_system_init();
    line_controller = chassis_system_get_line_controller();
    yaw_controller = chassis_system_get_yaw_controller();
    next = xTaskGetTickCount();
    reset_main_context(&ctx, line_controller, yaw_controller);

    for (;;) {
        app_challenge_info_t challenge;
        chassis_feedback_t feedback;
        chassis_command_t requested_command;
        chassis_command_t command = {0};
        const phase_descriptor_t *phase;
        float target_v_mps = 0.0f;
        app_mode_t mode;

        app_state_get_challenge(&challenge);
        app_state_get_feedback(&feedback);
        app_state_get_command(&requested_command);
        mode = app_state_get_mode();

        if ((mode == APP_MODE_STRAIGHT_TEST) || (mode == APP_MODE_LINE_TEST)) {
            if ((ctx.active == 0u) || (ctx.active_mode != mode)) {
                enter_basic_mode(&ctx,
                                 mode,
                                 &requested_command,
                                 &feedback,
                                 line_controller,
                                 yaw_controller);
            }

            if (mode == APP_MODE_STRAIGHT_TEST) {
                run_basic_straight(&ctx,
                                   yaw_controller,
                                   &feedback,
                                   &command);
                if (ctx.phase_distance_m >= ctx.target_distance_m) {
                    stop_basic_mode(&ctx, line_controller, yaw_controller);
                    vTaskDelayUntil(&next, pdMS_TO_TICKS(MAIN_TASK_PERIOD_MS));
                    continue;
                }
                publish_basic_runtime_state(&ctx,
                                            APP_PHASE_ACTION_GAP_TRAVERSE,
                                            APP_CHALLENGE_PHASE_GAP_AB,
                                            APP_CHALLENGE_STATUS_RUNNING);
            } else {
                run_basic_line(&ctx,
                               line_controller,
                               &feedback,
                               &command);
                if ((ctx.target_distance_m > 0.0f) &&
                    (ctx.phase_distance_m >= ctx.target_distance_m)) {
                    stop_basic_mode(&ctx, line_controller, yaw_controller);
                    vTaskDelayUntil(&next, pdMS_TO_TICKS(MAIN_TASK_PERIOD_MS));
                    continue;
                }
                publish_basic_runtime_state(&ctx,
                                            APP_PHASE_ACTION_ARC_TRACK,
                                            APP_CHALLENGE_PHASE_ARC_BC,
                                            APP_CHALLENGE_STATUS_RUNNING);
            }

            app_state_set_command(&command);
            vTaskDelayUntil(&next, pdMS_TO_TICKS(MAIN_TASK_PERIOD_MS));
            continue;
        }

        if ((mode != APP_MODE_MAIN) ||
            (challenge.active == APP_CHALLENGE_NONE) ||
            ((challenge.status != APP_CHALLENGE_STATUS_ALIGN) &&
             (challenge.status != APP_CHALLENGE_STATUS_RUNNING))) {
            if (ctx.active != 0u) {
                reset_main_context(&ctx, line_controller, yaw_controller);
            }
            vTaskDelayUntil(&next, pdMS_TO_TICKS(MAIN_TASK_PERIOD_MS));
            continue;
        }

        if ((ctx.active == 0u) || (ctx.challenge_active != challenge.active)) {
            const phase_descriptor_t *first_phase;

            reset_main_context(&ctx, line_controller, yaw_controller);
            ctx.active = 1u;
            ctx.active_mode = APP_MODE_MAIN;
            ctx.challenge_active = challenge.active;
            ctx.sequence_index = 0u;
            ctx.lap_total = app_challenge_lap_total(challenge.active);
            ctx.lap_index = 1u;
            ctx.checkpoint_count = 0u;

            first_phase = get_phase_descriptor(ctx.challenge_active, ctx.sequence_index);
            if (first_phase == NULL) {
                finish_challenge(&ctx, line_controller, yaw_controller, &challenge, NULL);
                vTaskDelayUntil(&next, pdMS_TO_TICKS(MAIN_TASK_PERIOD_MS));
                continue;
            }
            ctx.field_yaw_offset_deg =
                wrap_angle_deg(feedback.yaw_deg -
                               geometry_to_imu_heading_deg(first_phase->geometry_heading_deg));
            enter_phase(&ctx, first_phase, &feedback, line_controller, yaw_controller);
        }

        phase = get_phase_descriptor(ctx.challenge_active, ctx.sequence_index);
        if (phase == NULL) {
            finish_challenge(&ctx, line_controller, yaw_controller, &challenge, NULL);
            vTaskDelayUntil(&next, pdMS_TO_TICKS(MAIN_TASK_PERIOD_MS));
            continue;
        }

        ctx.phase_distance_m = get_phase_distance_m(&ctx, &feedback);

        command.stop = 0u;
        command.enable_closed_loop = 1u;

        switch (phase->action) {
        case APP_PHASE_ACTION_ALIGN_START:
            command.stop = 1u;
            command.enable_closed_loop = 0u;
            ctx.target_speed_mps = 0.0f;
            ctx.current_v_mps = 0.0f;
            ctx.heading_error_deg =
                wrap_angle_deg(get_field_heading_deg(&ctx, phase) - feedback.yaw_deg);
            ctx.state = APP_MAIN_STATE_ALIGN;

            if ((feedback.imu_ready != 0u) &&
                (feedback.imu_stable != 0u) &&
                (absf(feedback.left_speed_mps) <= MAIN_ALIGN_SPEED_THRESHOLD_MPS) &&
                (absf(feedback.right_speed_mps) <= MAIN_ALIGN_SPEED_THRESHOLD_MPS)) {
                if (ctx.align_stable_ms < MAIN_ALIGN_STABLE_MS) {
                    ctx.align_stable_ms = (uint16_t)(ctx.align_stable_ms + MAIN_TASK_PERIOD_MS);
                }
            } else {
                ctx.align_stable_ms = 0u;
            }

            if (ctx.align_stable_ms >= MAIN_ALIGN_STABLE_MS) {
                uint8_t next_sequence_index;
                uint8_t next_lap_index;

                app_state_emit_event(APP_EVENT_START);
                if (resolve_next_sequence(&ctx, &next_sequence_index, &next_lap_index) == 0u) {
                    finish_challenge(&ctx, line_controller, yaw_controller, &challenge, NULL);
                    vTaskDelayUntil(&next, pdMS_TO_TICKS(MAIN_TASK_PERIOD_MS));
                    continue;
                }

                ctx.sequence_index = next_sequence_index;
                ctx.lap_index = next_lap_index;
                phase = get_phase_descriptor(ctx.challenge_active, ctx.sequence_index);
                if (phase == NULL) {
                    finish_challenge(&ctx, line_controller, yaw_controller, &challenge, NULL);
                    vTaskDelayUntil(&next, pdMS_TO_TICKS(MAIN_TASK_PERIOD_MS));
                    continue;
                }
                enter_phase(&ctx, phase, &feedback, line_controller, yaw_controller);
            }
            break;

        case APP_PHASE_ACTION_GAP_TRAVERSE: {
            uint8_t next_sequence_index = 0u;
            uint8_t next_lap_index = ctx.lap_index;
            const phase_descriptor_t *next_phase = NULL;
            uint8_t have_next_phase;
            uint8_t d_center_required;
            uint16_t confirm_ms;
            uint8_t line_exit_ready;
            uint8_t encoder_fallback_ready;

            have_next_phase =
                resolve_next_phase_descriptor(&ctx,
                                              &next_sequence_index,
                                              &next_lap_index,
                                              &next_phase);
            d_center_required = phase_requires_centered_d_exit(phase);
            confirm_ms = (d_center_required != 0u)
                             ? select_gap_d_confirm_ms(phase)
                             : MAIN_REACQUIRE_CONFIRM_MS;

            target_v_mps = run_gap_traverse(&ctx, phase, yaw_controller, &feedback, &command);
            if ((d_center_required != 0u) && (ctx.phase_distance_m >= phase->min_exit_m)) {
                target_v_mps = clampf(target_v_mps, 0.0f, MAIN_GAP_D_EXIT_SPEED_MPS);
                apply_gap_d_exit_centering(phase, line_controller, &feedback, &command);
            } else {
                LineController_Reset(line_controller);
            }

            if ((gap_arc_encoder_fallback_window(phase,
                                                 next_phase,
                                                 ctx.phase_distance_m) != 0u) &&
                (feedback.line_detected == 0u)) {
                target_v_mps = clampf(target_v_mps,
                                      0.0f,
                                      MAIN_GAP_ARC_FALLBACK_SPEED_MPS);
            }
            ctx.target_speed_mps = target_v_mps;

            if (feedback.line_detected == 0u) {
                ctx.gap_left_line = 1u;
                ctx.line_seen_ms = 0u;
            } else if ((ctx.gap_left_line != 0u) &&
                       (ctx.phase_distance_m >= phase->min_exit_m)) {
                if ((d_center_required == 0u) ||
                    (feedback_is_centered_for_d_exit(phase, &feedback) != 0u)) {
                    ctx.line_seen_ms =
                        (uint16_t)(ctx.line_seen_ms + MAIN_TASK_PERIOD_MS);
                    if (ctx.line_seen_ms > confirm_ms) {
                        ctx.line_seen_ms = confirm_ms;
                    }
                } else {
                    ctx.line_seen_ms = 0u;
                }
            } else {
                ctx.line_seen_ms = 0u;
            }

            line_exit_ready = (ctx.line_seen_ms >= confirm_ms) ? 1u : 0u;
            encoder_fallback_ready =
                ((have_next_phase != 0u) &&
                 (gap_arc_encoder_fallback_ready(phase,
                                                 next_phase,
                                                 ctx.phase_distance_m) != 0u))
                    ? 1u
                    : 0u;

            if ((line_exit_ready != 0u) || (encoder_fallback_ready != 0u)) {
                ctx.checkpoint_count = (uint8_t)(ctx.checkpoint_count + 1u);
                app_state_emit_event(phase->complete_event);

                if (have_next_phase == 0u) {
                    finish_challenge(&ctx, line_controller, yaw_controller, &challenge, NULL);
                    vTaskDelayUntil(&next, pdMS_TO_TICKS(MAIN_TASK_PERIOD_MS));
                    continue;
                }

                if (next_phase->action == APP_PHASE_ACTION_STOP_AND_SIGNAL) {
                    ctx.lap_index = next_lap_index;
                    finish_challenge(&ctx, line_controller, yaw_controller, &challenge, next_phase);
                    vTaskDelayUntil(&next, pdMS_TO_TICKS(MAIN_TASK_PERIOD_MS));
                    continue;
                }

                ctx.sequence_index = next_sequence_index;
                ctx.lap_index = next_lap_index;
                phase = next_phase;
                enter_phase(&ctx, phase, &feedback, line_controller, yaw_controller);
            }
            break;
        }

        case APP_PHASE_ACTION_ARC_TRACK: {
            uint8_t next_sequence_index = 0u;
            uint8_t next_lap_index = ctx.lap_index;
            const phase_descriptor_t *next_phase = NULL;
            uint8_t have_next_phase;

            have_next_phase =
                resolve_next_phase_descriptor(&ctx,
                                              &next_sequence_index,
                                              &next_lap_index,
                                              &next_phase);

            target_v_mps = run_arc_track(&ctx, phase, line_controller, &feedback, &command);
            target_v_mps = apply_final_a_speed_limit(phase,
                                                     next_phase,
                                                     target_v_mps,
                                                     ctx.phase_distance_m);
            ctx.target_speed_mps = target_v_mps;

            if (feedback.line_detected == 0u) {
                ctx.line_missing_ms = (uint16_t)(ctx.line_missing_ms + MAIN_TASK_PERIOD_MS);
            } else {
                ctx.line_missing_ms = 0u;
            }

            if ((ctx.phase_distance_m >= phase->arc_profile->min_exit_m) &&
                ((ctx.line_missing_ms >= phase->arc_profile->lost_confirm_ms) ||
                 (ctx.final_a_compensating != 0u))) {
                if (have_next_phase == 0u) {
                    finish_challenge(&ctx, line_controller, yaw_controller, &challenge, NULL);
                    vTaskDelayUntil(&next, pdMS_TO_TICKS(MAIN_TASK_PERIOD_MS));
                    continue;
                }

                if (next_phase->action == APP_PHASE_ACTION_STOP_AND_SIGNAL) {
                    if ((phase_is_final_a_arc(phase, next_phase) != 0u) &&
                        (run_final_a_compensate(&ctx,
                                                phase,
                                                yaw_controller,
                                                &feedback,
                                                &command) == 0u)) {
                        break;
                    }

                    ctx.lap_index = next_lap_index;
                    finish_challenge(&ctx, line_controller, yaw_controller, &challenge, next_phase);
                    vTaskDelayUntil(&next, pdMS_TO_TICKS(MAIN_TASK_PERIOD_MS));
                    continue;
                }

                if (run_arc_exit_heading_align(&ctx,
                                               next_phase,
                                               yaw_controller,
                                               &feedback,
                                               &command) == 0u) {
                    break;
                }

                ctx.checkpoint_count = (uint8_t)(ctx.checkpoint_count + 1u);
                app_state_emit_event(phase->complete_event);

                ctx.sequence_index = next_sequence_index;
                ctx.lap_index = next_lap_index;
                phase = next_phase;
                enter_phase(&ctx, phase, &feedback, line_controller, yaw_controller);
            }
            break;
        }

        case APP_PHASE_ACTION_STOP_AND_SIGNAL:
            finish_challenge(&ctx, line_controller, yaw_controller, &challenge, phase);
            vTaskDelayUntil(&next, pdMS_TO_TICKS(MAIN_TASK_PERIOD_MS));
            continue;

        case APP_PHASE_ACTION_NONE:
        default:
            finish_challenge(&ctx, line_controller, yaw_controller, &challenge, NULL);
            vTaskDelayUntil(&next, pdMS_TO_TICKS(MAIN_TASK_PERIOD_MS));
            continue;
        }

        ctx.current_v_mps = apply_speed_slew(ctx.current_v_mps, ctx.target_speed_mps);
        command.v_mps = ctx.current_v_mps;

        publish_runtime_state(&ctx,
                              &challenge,
                              phase,
                              (phase->action == APP_PHASE_ACTION_ALIGN_START)
                                  ? APP_CHALLENGE_STATUS_ALIGN
                                  : APP_CHALLENGE_STATUS_RUNNING);
        app_state_set_command(&command);
        vTaskDelayUntil(&next, pdMS_TO_TICKS(MAIN_TASK_PERIOD_MS));
    }
}
