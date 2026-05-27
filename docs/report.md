# 核心逻辑代码节选

下面给出本项目最核心的 3 段控制代码，分别对应赛题路径描述、直线/圆弧控制切换，以及底盘与控制器初始化。

## 1. 赛题状态机与 Q4 多圈路径描述

来源：`Sources/tasks/main_task.c`

```c
static const phase_descriptor_t k_q4_sequence[] = {
    {APP_CHALLENGE_PHASE_ALIGN_A_TO_C, APP_PHASE_ACTION_ALIGN_START,  MAIN_HEADING_AC_DEG, 0.0f,   0.0f,   APP_EVENT_NONE,   NULL},
    {APP_CHALLENGE_PHASE_GAP_AC,       APP_PHASE_ACTION_GAP_TRAVERSE, MAIN_HEADING_AC_DEG, 1.281f, 1.03f,  APP_EVENT_PASS_C, NULL},
    {APP_CHALLENGE_PHASE_ARC_CB,       APP_PHASE_ACTION_ARC_TRACK,    0.0f,                 0.0f,   0.0f,   APP_EVENT_PASS_B, &k_arc_profile_cb},
    {APP_CHALLENGE_PHASE_GAP_BD,       APP_PHASE_ACTION_GAP_TRAVERSE, MAIN_HEADING_BD_DEG, 1.281f, 1.03f,  APP_EVENT_PASS_D, NULL},
    {APP_CHALLENGE_PHASE_ARC_DA,       APP_PHASE_ACTION_ARC_TRACK,    0.0f,                 0.0f,   0.0f,   APP_EVENT_PASS_A, &k_arc_profile_da_q3q4},
    {APP_CHALLENGE_PHASE_STOP_A,       APP_PHASE_ACTION_STOP_AND_SIGNAL, 0.0f,              0.0f,   0.0f,   APP_EVENT_STOP,   NULL},
};

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
```

## 2. 直线段与圆弧段控制切换

来源：`Sources/tasks/main_task.c`

```c
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
    zone_speed_mps = apply_arc_entry_speed_limit(phase, zone_speed_mps, ctx->phase_distance_m);

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
        return clampf(ctx->current_v_mps, MAIN_ARC_SEARCH_SPEED_MPS, zone_speed_mps);
    }
    return MAIN_ARC_SEARCH_SPEED_MPS;
}
```

## 3. 底盘与控制器初始化

来源：`Sources/chassis_system.c`

```c
void chassis_system_init(void)
{
    const wheel_cfg_t left_wheel_cfg = {
        .speed_pid = {
            .kp = 0.265742f, .ki = 3.321770f, .kd = 0.0f,
            .out_min = -1.0f, .out_max = 1.0f,
            .integral_min = -0.5f, .integral_max = 0.5f,
            .d_filter_alpha = 0.85f,
            .anti_windup_enable = 1u,
            .mode = PID_MODE_POSITIONAL,
        },
        .speed_ff_gain = 1.162620f,
    };
    const wheel_cfg_t right_wheel_cfg = {
        .speed_pid = {
            .kp = 0.265832f, .ki = 3.322895f, .kd = 0.0f,
            .out_min = -1.0f, .out_max = 1.0f,
            .integral_min = -0.5f, .integral_max = 0.5f,
            .d_filter_alpha = 0.85f,
            .anti_windup_enable = 1u,
            .mode = PID_MODE_POSITIONAL,
        },
        .speed_ff_gain = 1.163013f,
    };
    const pid_config_t yaw_pid_cfg = {
        .kp = 0.02f, .ki = 0.0f, .kd = 0.0005f,
        .out_min = -6.0f, .out_max = 6.0f,
        .integral_min = -2.0f, .integral_max = 2.0f,
        .d_filter_alpha = 0.90f,
        .anti_windup_enable = 1u,
        .mode = PID_MODE_POSITIONAL,
    };
    const pid_config_t line_pid_cfg = {
        .kp = 0.067f, .ki = 0.0f, .kd = 0.0004f,
        .out_min = -2.6f, .out_max = 2.6f,
        .integral_min = -2.0f, .integral_max = 2.0f,
        .d_filter_alpha = 0.92f,
        .anti_windup_enable = 1u,
        .mode = PID_MODE_POSITIONAL,
    };

    Motor_Init(&s_left_motor, &left_motor_cfg);
    Motor_Init(&s_right_motor, &right_motor_cfg);
    Encoder_Init(&s_left_encoder, &left_encoder_cfg);
    Encoder_Init(&s_right_encoder, &right_encoder_cfg);
    LineSensor_Init(&s_line_sensor);

    Wheel_Init(&s_left_wheel, &s_left_motor, &s_left_encoder, &left_wheel_cfg);
    Wheel_Init(&s_right_wheel, &s_right_motor, &s_right_encoder, &right_wheel_cfg);
    Chassis_Init(&s_chassis, &s_left_wheel, &s_right_wheel, 0.14f);
    YawController_Init(&s_yaw_controller, &yaw_pid_cfg, 0.0005f);
    LineController_Init(&s_line_controller, &line_pid_cfg);

    s_imu.ready = 0u;
    NVIC_EnableIRQ(TIMER_CALC_INST_INT_IRQN);
    DL_TimerA_startCounter(TIMER_CALC_INST);
    s_initialized = true;
}
```
