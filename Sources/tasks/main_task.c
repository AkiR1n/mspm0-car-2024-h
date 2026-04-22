#include "main_task.h"

#include "FreeRTOS.h"
#include "task.h"

#include "app_state.h"
#include "chassis_system.h"

#define MAIN_TASK_PERIOD_MS             10U
#define MAIN_DT_S                       0.01f
#define MAIN_FAST_SPEED_MPS             0.22f
#define MAIN_MEDIUM_SPEED_MPS           0.16f
#define MAIN_SLOW_SPEED_MPS             0.10f
#define MAIN_SEARCH_SPEED_MPS           0.06f
#define MAIN_SEARCH_W_RADPS             1.8f
#define MAIN_W_LIMIT_RADPS              2.5f
#define MAIN_FAST_POSITION_THRESHOLD    5
#define MAIN_MEDIUM_POSITION_THRESHOLD  15

typedef struct {
    uint8_t          active;
    uint8_t          has_seen_line;
    float            last_seen_error;
    app_main_state_t state;
} main_task_ctx_t;

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

static float select_track_speed(float line_error)
{
    float abs_error = absf(line_error);

    if (abs_error <= (float)MAIN_FAST_POSITION_THRESHOLD) {
        return MAIN_FAST_SPEED_MPS;
    }
    if (abs_error <= (float)MAIN_MEDIUM_POSITION_THRESHOLD) {
        return MAIN_MEDIUM_SPEED_MPS;
    }
    return MAIN_SLOW_SPEED_MPS;
}

static void reset_main_context(main_task_ctx_t *ctx, line_controller_t *line_controller)
{
    if (ctx == NULL) {
        return;
    }

    ctx->active = 0u;
    ctx->has_seen_line = 0u;
    ctx->last_seen_error = 0.0f;
    ctx->state = APP_MAIN_STATE_IDLE;
    LineController_Reset(line_controller);
    app_state_set_main_state(APP_MAIN_STATE_IDLE);
}

void main_task(void *arg)
{
    TickType_t next;
    main_task_ctx_t ctx;
    line_controller_t *line_controller;

    (void)arg;

    chassis_system_init();
    line_controller = chassis_system_get_line_controller();
    next = xTaskGetTickCount();
    reset_main_context(&ctx, line_controller);

    for (;;) {
        if (app_state_get_mode() != APP_MODE_MAIN) {
            if (ctx.active != 0u) {
                reset_main_context(&ctx, line_controller);
            }
            vTaskDelayUntil(&next, pdMS_TO_TICKS(MAIN_TASK_PERIOD_MS));
            continue;
        }

        if (ctx.active == 0u) {
            ctx.active = 1u;
            ctx.state = APP_MAIN_STATE_IDLE;
            LineController_Reset(line_controller);
            app_state_set_main_state(APP_MAIN_STATE_IDLE);
        }

        {
            chassis_feedback_t feedback;
            chassis_command_t command = {0};

            app_state_get_feedback(&feedback);

            command.stop = 0u;
            command.enable_closed_loop = 1u;

            if (feedback.line_detected != 0u) {
                float error = (float)feedback.line_position;

                command.v_mps = select_track_speed(error);
                command.w_radps = clampf(
                    LineController_Update(line_controller,
                                          error,
                                          feedback.line_bits,
                                          feedback.line_detected,
                                          MAIN_DT_S),
                    -MAIN_W_LIMIT_RADPS,
                    MAIN_W_LIMIT_RADPS);
                ctx.has_seen_line = 1u;
                ctx.last_seen_error = error;
                ctx.state = APP_MAIN_STATE_TRACK;
            } else {
                LineController_Reset(line_controller);
                command.v_mps = MAIN_SEARCH_SPEED_MPS;

                if ((ctx.has_seen_line == 0u) || (ctx.last_seen_error <= 0.0f)) {
                    command.w_radps = MAIN_SEARCH_W_RADPS;
                    ctx.state = APP_MAIN_STATE_LOST_LEFT;
                } else {
                    command.w_radps = -MAIN_SEARCH_W_RADPS;
                    ctx.state = APP_MAIN_STATE_LOST_RIGHT;
                }
            }

            app_state_set_main_state(ctx.state);
            app_state_set_command(&command);
        }

        vTaskDelayUntil(&next, pdMS_TO_TICKS(MAIN_TASK_PERIOD_MS));
    }
}
