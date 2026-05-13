#include "test_task.h"

#include <ctype.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app_state.h"
#include "bt_uart.h"
#include "chassis_system.h"
#include "encoder_hal.h"
#include "imu_drv.h"
#include "linetracker.h"
#include "pid.h"
#include "uart_printf.h"
#include "uart_rx.h"

#define MODE_TASK_PERIOD_MS          50U
#define MODE_CMD_BUFFER_SIZE         96U
#define MODE_AUTO_PHASE_SIZE         32U
#define MODE_REPORT_PERIOD_MS        50U
#define MODE_DUTY_SCALE              0.01f
#define MODE_KEY_COUNT               2U
#define MODE_KEY_DEBOUNCE_SAMPLES    2U

typedef struct {
    GPIO_Regs *port;
    uint32_t   pin;
    const char *name;
} mode_key_desc_t;

typedef struct {
    uint8_t raw_pressed;
    uint8_t stable_pressed;
    uint8_t same_count;
} mode_key_state_t;

typedef struct {
    char     rx_line[MODE_CMD_BUFFER_SIZE];
    size_t   rx_len;
    char     bt_rx_line[MODE_CMD_BUFFER_SIZE];
    size_t   bt_rx_len;
    char     auto_phase[MODE_AUTO_PHASE_SIZE];
    uint32_t last_report_tick_ms;
    uint32_t last_irq_count;
    uint32_t last_event_seq_seen;
    uint8_t  auto_stream_enabled;
    mode_key_state_t keys[MODE_KEY_COUNT];
} test_task_ctx_t;

static uint8_t s_imu_only_mode = 0u;

static const mode_key_desc_t k_mode_keys[MODE_KEY_COUNT] = {
    {GPIO_Switch_Key_1_PORT, GPIO_Switch_Key_1_PIN, "key1"},
    {GPIO_Switch_Key_2_PORT, GPIO_Switch_Key_2_PIN, "key2"},
};

static void format_line_bits(uint8_t bits, char out[8])
{
    uint8_t i;

    if (out == NULL) {
        return;
    }

    for (i = 0u; i < 7u; ++i) {
        out[i] = ((bits & (1u << i)) != 0u) ? '1' : '0';
    }
    out[7] = '\0';
}

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

static char *trim_ascii(char *text)
{
    char *end;

    while ((*text != '\0') && isspace((unsigned char)*text)) {
        ++text;
    }

    end = text + strlen(text);
    while ((end > text) && isspace((unsigned char)end[-1])) {
        --end;
    }
    *end = '\0';
    return text;
}

static void sanitize_phase_label(char *dst, size_t dst_size, const char *src)
{
    size_t dst_index = 0u;

    if ((dst == NULL) || (dst_size == 0u)) {
        return;
    }

    if (src == NULL) {
        dst[0] = '\0';
        return;
    }

    while ((*src != '\0') && (dst_index + 1u < dst_size)) {
        char ch = *src++;

        if ((ch == ',') || (ch == '\r') || (ch == '\n')) {
            continue;
        }
        if (isspace((unsigned char)ch)) {
            ch = '_';
        }
        dst[dst_index++] = ch;
    }
    dst[dst_index] = '\0';
}

static bool parse_float_pair(const char *text, float *left, float *right)
{
    char *endptr;
    float left_value;
    float right_value;

    if ((text == NULL) || (left == NULL) || (right == NULL)) {
        return false;
    }

    while (isspace((unsigned char)*text)) {
        ++text;
    }
    if (*text == '(') {
        ++text;
    }

    left_value = strtof(text, &endptr);
    if (endptr == text) {
        return false;
    }

    text = endptr;
    while (isspace((unsigned char)*text)) {
        ++text;
    }
    if (*text != ',') {
        return false;
    }
    ++text;

    while (isspace((unsigned char)*text)) {
        ++text;
    }
    right_value = strtof(text, &endptr);
    if (endptr == text) {
        return false;
    }

    text = endptr;
    while (isspace((unsigned char)*text)) {
        ++text;
    }
    if (*text == ')') {
        ++text;
    }
    while (isspace((unsigned char)*text)) {
        ++text;
    }
    if (*text != '\0') {
        return false;
    }

    *left = left_value;
    *right = right_value;
    return true;
}

static void clear_challenge_runtime(app_challenge_info_t *challenge,
                                    app_challenge_status_t status,
                                    uint8_t clear_events)
{
    if (challenge == NULL) {
        return;
    }

    challenge->active = APP_CHALLENGE_NONE;
    challenge->status = status;
    challenge->phase = APP_CHALLENGE_PHASE_IDLE;
    challenge->action = APP_PHASE_ACTION_NONE;
    challenge->checkpoint_count = 0u;
    challenge->lap_total = app_challenge_lap_total(challenge->selected);
    challenge->lap_index = (challenge->lap_total != 0u) ? 1u : 0u;
    challenge->geometry_heading_deg = 0.0f;
    challenge->hold_heading_deg = 0.0f;
    challenge->heading_error_deg = 0.0f;
    challenge->phase_distance_m = 0.0f;
    challenge->target_speed_mps = 0.0f;

    if (clear_events != 0u) {
        challenge->last_event = APP_EVENT_NONE;
        challenge->last_event_ms = 0u;
        challenge->event_seq = 0u;
    }
}

static void apply_stop_command(void)
{
    app_challenge_info_t challenge;
    chassis_command_t command = {0};
    uint8_t should_emit_stop = 0u;

    app_state_get_challenge(&challenge);
    should_emit_stop = (uint8_t)((challenge.status == APP_CHALLENGE_STATUS_ALIGN) ||
                                 (challenge.status == APP_CHALLENGE_STATUS_RUNNING));
    clear_challenge_runtime(&challenge, APP_CHALLENGE_STATUS_READY, 0u);

    if (should_emit_stop != 0u) {
        app_state_emit_event(APP_EVENT_STOP);
        app_state_get_challenge(&challenge);
        clear_challenge_runtime(&challenge, APP_CHALLENGE_STATUS_READY, 0u);
    }

    app_state_set_challenge(&challenge);

    command.stop = 1u;
    command.enable_closed_loop = 0u;
    app_state_set_mode(APP_MODE_STOP);
    app_state_set_command(&command);
}

static void apply_main_command(void)
{
    chassis_command_t command = {0};

    command.stop = 1u;
    command.enable_closed_loop = 0u;
    app_state_set_mode(APP_MODE_MAIN);
    app_state_set_command(&command);
}

static void select_challenge(app_challenge_t challenge_id)
{
    app_challenge_info_t challenge;

    app_state_get_challenge(&challenge);
    challenge.selected = challenge_id;
    clear_challenge_runtime(&challenge, APP_CHALLENGE_STATUS_READY, 1u);
    app_state_set_challenge(&challenge);
}

static bool start_selected_challenge(void)
{
    app_challenge_info_t challenge;

    app_state_get_challenge(&challenge);

    if (challenge.selected == APP_CHALLENGE_NONE) {
        return false;
    }

    challenge.active = challenge.selected;
    challenge.status = APP_CHALLENGE_STATUS_ALIGN;
    challenge.phase = APP_CHALLENGE_PHASE_IDLE;
    challenge.action = APP_PHASE_ACTION_NONE;
    challenge.checkpoint_count = 0u;
    challenge.lap_total = app_challenge_lap_total(challenge.selected);
    challenge.lap_index = (challenge.lap_total != 0u) ? 1u : 0u;
    challenge.geometry_heading_deg = 0.0f;
    challenge.hold_heading_deg = 0.0f;
    challenge.heading_error_deg = 0.0f;
    challenge.phase_distance_m = 0.0f;
    challenge.target_speed_mps = 0.0f;
    challenge.last_event = APP_EVENT_NONE;
    challenge.last_event_ms = 0u;
    challenge.event_seq = 0u;
    app_state_set_challenge(&challenge);
    apply_main_command();
    return true;
}

static void apply_wheel_duty_command(float left_percent, float right_percent)
{
    chassis_command_t command = {0};

    command.stop = 0u;
    command.enable_closed_loop = 0u;
    command.left_duty = clampf(left_percent * MODE_DUTY_SCALE, -1.0f, 1.0f);
    command.right_duty = clampf(right_percent * MODE_DUTY_SCALE, -1.0f, 1.0f);
    app_state_set_mode(APP_MODE_WHEEL_TEST);
    app_state_set_command(&command);
}

static void apply_speed_command(float left_mps, float right_mps)
{
    chassis_command_t command = {0};

    command.stop = 0u;
    command.enable_closed_loop = 1u;
    command.left_speed_mps = left_mps;
    command.right_speed_mps = right_mps;
    app_state_set_mode(APP_MODE_WHEEL_SPEED_TEST);
    app_state_set_command(&command);
}

static void apply_twist_command(float v_mps, float w_radps)
{
    chassis_command_t command = {0};

    command.stop = 0u;
    command.enable_closed_loop = 1u;
    command.v_mps = v_mps;
    command.w_radps = w_radps;
    app_state_set_mode(APP_MODE_TWIST_OPEN);
    app_state_set_command(&command);
}

static void print_pid_line(const char *label, int wheel_index)
{
    pid_config_t pid_cfg;
    float ff_gain;
    const char *mode_name;

    chassis_system_get_wheel_pid_config(wheel_index, &pid_cfg);
    ff_gain = chassis_system_get_wheel_ff_gain(wheel_index);
    mode_name = (pid_cfg.mode == PID_MODE_INCREMENTAL) ? "inc" : "pos";

    uart_printf(
        "pid[%s] kp=%.4f ki=%.4f kd=%.4f ff=%.4f mode=%s alpha=%.3f out=(%.3f,%.3f) i=(%.3f,%.3f)\r\n",
        label,
        pid_cfg.kp,
        pid_cfg.ki,
        pid_cfg.kd,
        ff_gain,
        mode_name,
        pid_cfg.d_filter_alpha,
        pid_cfg.out_min,
        pid_cfg.out_max,
        pid_cfg.integral_min,
        pid_cfg.integral_max);
}

static void emit_auto_pid_line(const char *label, int wheel_index)
{
    pid_config_t pid_cfg;
    float ff_gain;
    const char *mode_name;

    chassis_system_get_wheel_pid_config(wheel_index, &pid_cfg);
    ff_gain = chassis_system_get_wheel_ff_gain(wheel_index);
    mode_name = (pid_cfg.mode == PID_MODE_INCREMENTAL) ? "inc" : "pos";

    uart_printf(
        "auto,pid,%s,%.6f,%.6f,%.6f,%.6f,%s,%.6f,%.6f,%.6f,%.6f,%.6f\r\n",
        label,
        pid_cfg.kp,
        pid_cfg.ki,
        pid_cfg.kd,
        ff_gain,
        mode_name,
        pid_cfg.d_filter_alpha,
        pid_cfg.out_min,
        pid_cfg.out_max,
        pid_cfg.integral_min,
        pid_cfg.integral_max);
}

static bool parse_pid_values(const char *payload, float values[4], unsigned *count_out)
{
    unsigned count = 0u;
    const char *cursor = payload;

    if ((payload == NULL) || (values == NULL) || (count_out == NULL)) {
        return false;
    }

    while (*cursor != '\0') {
        char *endptr;

        while (isspace((unsigned char)*cursor)) {
            ++cursor;
        }
        if (*cursor == '\0') {
            break;
        }
        if (count >= 4u) {
            return false;
        }

        values[count] = strtof(cursor, &endptr);
        if (endptr == cursor) {
            return false;
        }

        ++count;
        cursor = endptr;

        while (isspace((unsigned char)*cursor)) {
            ++cursor;
        }
        if (*cursor == '\0') {
            break;
        }
        if (*cursor != ',') {
            return false;
        }
        ++cursor;
    }

    if (count < 3u) {
        return false;
    }

    *count_out = count;
    return true;
}

static void apply_pid_values_to_wheel(int wheel_index,
                                      const float values[4],
                                      unsigned count)
{
    pid_config_t pid_cfg;

    chassis_system_get_wheel_pid_config(wheel_index, &pid_cfg);

    pid_cfg.kp = values[0];
    pid_cfg.ki = values[1];
    pid_cfg.kd = values[2];
    chassis_system_set_wheel_pid_config(wheel_index, &pid_cfg);
    if (count >= 4u) {
        chassis_system_set_wheel_ff_gain(wheel_index, values[3]);
    }
}

static uint32_t calc_irq_per_second(test_task_ctx_t *ctx, uint32_t now_ms)
{
    uint32_t irq_count;
    uint32_t delta_irq;
    uint32_t elapsed_ms;
    uint32_t irq_per_s = 0u;

    irq_count = EncoderHal_GetGpioIrqCount();
    elapsed_ms = now_ms - ctx->last_report_tick_ms;
    delta_irq = irq_count - ctx->last_irq_count;

    if (elapsed_ms > 0u) {
        irq_per_s = (delta_irq * 1000u) / elapsed_ms;
    }

    ctx->last_report_tick_ms = now_ms;
    ctx->last_irq_count = irq_count;
    return irq_per_s;
}

static void emit_auto_sample(test_task_ctx_t *ctx, uint32_t now_ms, uint32_t irq_per_s)
{
    app_state_snapshot_t snapshot;
    const char *phase;

    app_state_get_snapshot(&snapshot);
    phase = (ctx->auto_phase[0] == '\0') ? "-" : ctx->auto_phase;

    uart_printf(
        "auto,sample,%lu,%s,%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%ld,%ld,%lu,%lu\r\n",
        (unsigned long)now_ms,
        phase,
        app_mode_name(snapshot.mode),
        snapshot.command.left_duty,
        snapshot.command.right_duty,
        snapshot.command.left_speed_mps,
        snapshot.command.right_speed_mps,
        snapshot.debug.left_target_mps,
        snapshot.debug.right_target_mps,
        snapshot.feedback.left_speed_mps,
        snapshot.feedback.right_speed_mps,
        snapshot.debug.left_motor_duty,
        snapshot.debug.right_motor_duty,
        (long)snapshot.feedback.left_count,
        (long)snapshot.feedback.right_count,
        (unsigned long)irq_per_s,
        (unsigned long)EncoderHal_GetSampleTickCount());
}

static void emit_imu_only(const app_state_snapshot_t *snapshot)
{
    imu_t *imu = chassis_system_get_imu();
    uart_printf(
        "imu: yaw=%.1f yaw_dmp=%.1f yaw_rel=%.1f "
        "gz=%.2f gz_raw=%.1f gz_bias=%.2f "
        "pitch=%.1f roll=%.1f "
        "rdy=%u stb=%u bias=%u up=%lu "
        "sign=%.0f sens=%.1f\r\n",
        snapshot->feedback.yaw_deg,
        (imu != NULL) ? imu->yaw_deg_raw : 0.0f,
        (imu != NULL) ? imu->yaw_rel_deg : 0.0f,
        snapshot->feedback.gyro_z,
        (imu != NULL) ? imu->gyro_z_raw : 0.0f,
        (imu != NULL) ? imu->gyro_z_bias : 0.0f,
        snapshot->feedback.pitch_deg,
        snapshot->feedback.roll_deg,
        (unsigned)snapshot->feedback.imu_ready,
        (unsigned)snapshot->feedback.imu_stable,
        (imu != NULL) ? imu->bias_committed : 0u,
        (unsigned long)snapshot->feedback.imu_uptime_ms,
        (imu != NULL) ? imu->cfg.gyro_z_sign : 0.0f,
        (imu != NULL) ? imu->gyro_sens_lsb_per_dps : 0.0f);
}

static void emit_human_status(uint32_t irq_per_s)
{
    app_state_snapshot_t snapshot;
    char line_text[8];

    app_state_get_snapshot(&snapshot);

    if (s_imu_only_mode != 0u) {
        emit_imu_only(&snapshot);
        return;
    }

    format_line_bits(snapshot.feedback.line_bits, line_text);

    switch (snapshot.mode) {
    case APP_MODE_WHEEL_SPEED_TEST:
        uart_printf(
            "mode=%s spd=(%.3f,%.3f) target=(%.3f,%.3f) meas=(%.3f,%.3f) duty=(%.3f,%.3f) count=(%ld,%ld) line=%s bits=0x%02X det=%u pos=%d irq/s=%lu tick=%lu\r\n",
            app_mode_name(snapshot.mode),
            snapshot.command.left_speed_mps,
            snapshot.command.right_speed_mps,
            snapshot.debug.left_target_mps,
            snapshot.debug.right_target_mps,
            snapshot.feedback.left_speed_mps,
            snapshot.feedback.right_speed_mps,
            snapshot.debug.left_motor_duty,
            snapshot.debug.right_motor_duty,
            (long)snapshot.feedback.left_count,
            (long)snapshot.feedback.right_count,
            line_text,
            (unsigned)snapshot.feedback.line_bits,
            (unsigned)snapshot.feedback.line_detected,
            (int)snapshot.feedback.line_position,
            (unsigned long)irq_per_s,
            (unsigned long)EncoderHal_GetSampleTickCount());
        break;
    case APP_MODE_WHEEL_TEST:
        uart_printf(
            "mode=%s duty=(%.3f,%.3f) meas=(%.3f,%.3f) count=(%ld,%ld) line=%s bits=0x%02X det=%u pos=%d irq/s=%lu tick=%lu\r\n",
            app_mode_name(snapshot.mode),
            snapshot.command.left_duty,
            snapshot.command.right_duty,
            snapshot.feedback.left_speed_mps,
            snapshot.feedback.right_speed_mps,
            (long)snapshot.feedback.left_count,
            (long)snapshot.feedback.right_count,
            line_text,
            (unsigned)snapshot.feedback.line_bits,
            (unsigned)snapshot.feedback.line_detected,
            (int)snapshot.feedback.line_position,
            (unsigned long)irq_per_s,
            (unsigned long)EncoderHal_GetSampleTickCount());
        break;
    case APP_MODE_TWIST_OPEN:
        uart_printf(
            "mode=%s vw=(%.3f,%.3f) target=(%.3f,%.3f) meas=(%.3f,%.3f) duty=(%.3f,%.3f) count=(%ld,%ld) line=%s bits=0x%02X det=%u pos=%d irq/s=%lu tick=%lu\r\n",
            app_mode_name(snapshot.mode),
            snapshot.command.v_mps,
            snapshot.command.w_radps,
            snapshot.debug.left_target_mps,
            snapshot.debug.right_target_mps,
            snapshot.feedback.left_speed_mps,
            snapshot.feedback.right_speed_mps,
            snapshot.debug.left_motor_duty,
            snapshot.debug.right_motor_duty,
            (long)snapshot.feedback.left_count,
            (long)snapshot.feedback.right_count,
            line_text,
            (unsigned)snapshot.feedback.line_bits,
            (unsigned)snapshot.feedback.line_detected,
            (int)snapshot.feedback.line_position,
            (unsigned long)irq_per_s,
            (unsigned long)EncoderHal_GetSampleTickCount());
        break;
    case APP_MODE_MAIN:
        uart_printf(
            "mode=%s q=%s/%s ph=%s act=%s lap=%u/%u cp=%u evt=%s@%lu state=%s hdg=(%.1f,%.1f,%.1f) dist=%.3f vw=(%.3f,%.3f) target=(%.3f,%.3f) meas=(%.3f,%.3f) duty=(%.3f,%.3f) line=%s bits=0x%02X det=%u pos=%d irq/s=%lu tick=%lu\r\n",
            app_mode_name(snapshot.mode),
            app_challenge_name(snapshot.challenge.selected),
            app_challenge_status_name(snapshot.challenge.status),
            app_challenge_phase_name(snapshot.challenge.phase),
            app_phase_action_name(snapshot.challenge.action),
            (unsigned)snapshot.challenge.lap_index,
            (unsigned)snapshot.challenge.lap_total,
            (unsigned)snapshot.challenge.checkpoint_count,
            app_event_name(snapshot.challenge.last_event),
            (unsigned long)snapshot.challenge.last_event_ms,
            app_main_state_name(snapshot.main_state),
            snapshot.challenge.geometry_heading_deg,
            snapshot.challenge.hold_heading_deg,
            snapshot.challenge.heading_error_deg,
            snapshot.challenge.phase_distance_m,
            snapshot.command.v_mps,
            snapshot.command.w_radps,
            snapshot.debug.left_target_mps,
            snapshot.debug.right_target_mps,
            snapshot.feedback.left_speed_mps,
            snapshot.feedback.right_speed_mps,
            snapshot.debug.left_motor_duty,
            snapshot.debug.right_motor_duty,
            line_text,
            (unsigned)snapshot.feedback.line_bits,
            (unsigned)snapshot.feedback.line_detected,
            (int)snapshot.feedback.line_position,
            (unsigned long)irq_per_s,
            (unsigned long)EncoderHal_GetSampleTickCount());
        break;
    case APP_MODE_STOP:
    default:
        uart_printf(
            "mode=%s stop=%u meas=(%.3f,%.3f) count=(%ld,%ld) line=%s bits=0x%02X det=%u pos=%d imu=(%u,%u) yaw=%.1f gz=%.2f up=%lu irq/s=%lu tick=%lu\r\n",
            app_mode_name(snapshot.mode),
            (unsigned)snapshot.command.stop,
            snapshot.feedback.left_speed_mps,
            snapshot.feedback.right_speed_mps,
            (long)snapshot.feedback.left_count,
            (long)snapshot.feedback.right_count,
            line_text,
            (unsigned)snapshot.feedback.line_bits,
            (unsigned)snapshot.feedback.line_detected,
            (int)snapshot.feedback.line_position,
            (unsigned)snapshot.feedback.imu_ready,
            (unsigned)snapshot.feedback.imu_stable,
            snapshot.feedback.yaw_deg,
            snapshot.feedback.gyro_z,
            (unsigned long)snapshot.feedback.imu_uptime_ms,
            (unsigned long)irq_per_s,
            (unsigned long)EncoderHal_GetSampleTickCount());
        break;
    }
}

static void emit_event_if_changed(test_task_ctx_t *ctx)
{
    app_challenge_info_t challenge;

    if (ctx == NULL) {
        return;
    }

    app_state_get_challenge(&challenge);
    if (challenge.event_seq < ctx->last_event_seq_seen) {
        ctx->last_event_seq_seen = challenge.event_seq;
        return;
    }
    if ((challenge.event_seq == 0u) ||
        (challenge.event_seq == ctx->last_event_seq_seen) ||
        (challenge.last_event == APP_EVENT_NONE)) {
        return;
    }

    ctx->last_event_seq_seen = challenge.event_seq;
    uart_printf("event=%s t=%lu q=%s ph=%s lap=%u/%u\r\n",
                app_event_name(challenge.last_event),
                (unsigned long)challenge.last_event_ms,
                app_challenge_name(challenge.selected),
                app_challenge_phase_name(challenge.phase),
                (unsigned)challenge.lap_index,
                (unsigned)challenge.lap_total);
}

static uint8_t read_gpio_level(GPIO_Regs *port, uint32_t pin)
{
    return (DL_GPIO_readPins(port, pin) != 0u) ? 1u : 0u;
}

static uint8_t read_switch_pressed(const mode_key_desc_t *key)
{
    if (key == NULL) {
        return 0u;
    }

    return (DL_GPIO_readPins(key->port, key->pin) == 0u) ? 1u : 0u;
}

static void handle_key_press(test_task_ctx_t *ctx, uint8_t key_index)
{
    app_challenge_info_t challenge;

    (void)ctx;

    app_state_get_challenge(&challenge);

    if (key_index == 0u) {
        if ((challenge.status == APP_CHALLENGE_STATUS_ALIGN) ||
            (challenge.status == APP_CHALLENGE_STATUS_RUNNING)) {
            uart_printf("%s busy=%s\r\n",
                        k_mode_keys[key_index].name,
                        app_challenge_name(challenge.active));
            return;
        }

        if (challenge.selected == APP_CHALLENGE_Q1) {
            select_challenge(APP_CHALLENGE_Q2);
        } else if (challenge.selected == APP_CHALLENGE_Q2) {
            select_challenge(APP_CHALLENGE_Q3);
        } else if (challenge.selected == APP_CHALLENGE_Q3) {
            select_challenge(APP_CHALLENGE_Q4);
        } else {
            select_challenge(APP_CHALLENGE_Q1);
        }

        app_state_get_challenge(&challenge);
        uart_printf("%s select=%s\r\n",
                    k_mode_keys[key_index].name,
                    app_challenge_name(challenge.selected));
        return;
    }

    if (key_index == 1u) {
        if ((challenge.status == APP_CHALLENGE_STATUS_ALIGN) ||
            (challenge.status == APP_CHALLENGE_STATUS_RUNNING)) {
            apply_stop_command();
            uart_printf("%s stop\r\n", k_mode_keys[key_index].name);
        } else if (start_selected_challenge()) {
            app_state_get_challenge(&challenge);
            uart_printf("%s run=%s\r\n",
                        k_mode_keys[key_index].name,
                        app_challenge_name(challenge.active));
        } else {
            uart_printf("%s start blocked: sel=%s\r\n",
                        k_mode_keys[key_index].name,
                        app_challenge_name(challenge.selected));
        }
    }
}

static void poll_keys(test_task_ctx_t *ctx)
{
    uint8_t i;

    for (i = 0u; i < MODE_KEY_COUNT; ++i) {
        uint8_t pressed = read_switch_pressed(&k_mode_keys[i]);
        mode_key_state_t *state = &ctx->keys[i];

        if (pressed == state->raw_pressed) {
            if (state->same_count < 0xFFu) {
                ++state->same_count;
            }
        } else {
            state->raw_pressed = pressed;
            state->same_count = 1u;
        }

        if ((state->same_count >= MODE_KEY_DEBOUNCE_SAMPLES) &&
            (state->stable_pressed != state->raw_pressed)) {
            state->stable_pressed = state->raw_pressed;
            if (state->stable_pressed != 0u) {
                handle_key_press(ctx, i);
            }
        }
    }
}

static void print_line_config(void)
{
    uart_printf("linecfg 0:%s 1:%s 2:%s 3:%s 4:%s 5:%s 6:%s\r\n",
                LineTracker_GetSensorPinName(0u),
                LineTracker_GetSensorPinName(1u),
                LineTracker_GetSensorPinName(2u),
                LineTracker_GetSensorPinName(3u),
                LineTracker_GetSensorPinName(4u),
                LineTracker_GetSensorPinName(5u),
                LineTracker_GetSensorPinName(6u));
    uart_printf("line_logic=%s\r\n", LineTracker_GetSensorLogicName());
}

static void print_line_raw_status(void)
{
    char raw_text[8];
    char norm_text[8];
    uint8_t raw_bits;
    uint8_t norm_bits;
    uint8_t i;

    LineTracker_ReadSensors();
    raw_bits = LineTracker_GetRawSensorBits();
    norm_bits = LineTracker_GetSensorBits();
    format_line_bits(raw_bits, raw_text);
    format_line_bits(norm_bits, norm_text);

    uart_printf("lineraw logic=%s raw=%s norm=%s bits=0x%02X det=%u pos=%d\r\n",
                LineTracker_GetSensorLogicName(),
                raw_text,
                norm_text,
                (unsigned)norm_bits,
                (unsigned)LineTracker_IsLineDetected(),
                (int)LineTracker_GetLinePosition());

    for (i = 0u; i < 7u; ++i) {
        uart_printf("linech[%u] pin=%s raw=%u norm=%u\r\n",
                    (unsigned)i,
                    LineTracker_GetSensorPinName(i),
                    (unsigned)LineTracker_GetRawSensorValue(i),
                    (unsigned)LineTracker_GetSensorValue(i));
    }

    uart_printf("lineaux pb18=%u pa15=%u\r\n",
                (unsigned)read_gpio_level(GPIO_Switch_Key_3_PORT, GPIO_Switch_Key_3_PIN),
                (unsigned)read_gpio_level(GPIO_LED_PIN_1_PORT, GPIO_LED_PIN_1_PIN));
}

static void print_help(void)
{
    uart_printf("test task ready\r\n");
    uart_printf("keys: key1=select_q1_q4 key2=run_or_stop\r\n");
    uart_printf("cmd: q1 | q2 | q3 | q4 | run | <left%%>,<right%%> | spd,<left_mps>,<right_mps> | twist,<v>,<w> | pid[|l|r],kp,ki,kd[,ff] | showpid | linepol,<0|1> | lineraw | linecfg[,reset|<idx>,<pin>] | main | stop | imu | imuz,<sign> | imus,<sens>\r\n");
    uart_printf("auto: auto,on|off | auto,phase,<label> | auto,duty,<l%%>,<r%%> | auto,spd,<l>,<r> | auto,pid | auto,pid,<left|right|both>,kp,ki,kd[,ff] | auto,sample | auto,stop\r\n");
    print_pid_line("left", 0);
    print_pid_line("right", 1);
    print_line_config();
}

static void handle_auto_command(test_task_ctx_t *ctx, char *payload)
{
    char *token;
    char *saveptr = NULL;

    token = strtok_r(payload, ",", &saveptr);
    if (token == NULL) {
        uart_printf("auto,err,empty\r\n");
        return;
    }
    token = trim_ascii(token);

    if (strcmp(token, "on") == 0) {
        ctx->auto_stream_enabled = 1u;
        uart_printf("auto,ack,on\r\n");
        return;
    }
    if (strcmp(token, "off") == 0) {
        ctx->auto_stream_enabled = 0u;
        uart_printf("auto,ack,off\r\n");
        return;
    }
    if (strcmp(token, "sample") == 0) {
        uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        uint32_t irq_per_s = calc_irq_per_second(ctx, now_ms);
        emit_auto_sample(ctx, now_ms, irq_per_s);
        return;
    }
    if (strcmp(token, "stop") == 0) {
        apply_stop_command();
        uart_printf("auto,ack,stop\r\n");
        return;
    }
    if (strcmp(token, "phase") == 0) {
        char *label = saveptr;
        sanitize_phase_label(ctx->auto_phase,
                             sizeof(ctx->auto_phase),
                             (label == NULL) ? "" : trim_ascii(label));
        uart_printf("auto,ack,phase,%s\r\n",
                    (ctx->auto_phase[0] == '\0') ? "-" : ctx->auto_phase);
        return;
    }
    if (strcmp(token, "duty") == 0) {
        float left_percent;
        float right_percent;
        char *pair = saveptr;

        if ((pair == NULL) || !parse_float_pair(trim_ascii(pair), &left_percent, &right_percent)) {
            uart_printf("auto,err,duty\r\n");
            return;
        }

        apply_wheel_duty_command(left_percent, right_percent);
        uart_printf("auto,ack,duty,%.3f,%.3f\r\n", left_percent, right_percent);
        return;
    }
    if ((strcmp(token, "spd") == 0) || (strcmp(token, "speed") == 0)) {
        float left_mps;
        float right_mps;
        char *pair = saveptr;

        if ((pair == NULL) || !parse_float_pair(trim_ascii(pair), &left_mps, &right_mps)) {
            uart_printf("auto,err,spd\r\n");
            return;
        }

        apply_speed_command(left_mps, right_mps);
        uart_printf("auto,ack,spd,%.6f,%.6f\r\n", left_mps, right_mps);
        return;
    }
    if (strcmp(token, "pid") == 0) {
        char *selector = strtok_r(NULL, ",", &saveptr);

        if (selector == NULL) {
            emit_auto_pid_line("left", 0);
            emit_auto_pid_line("right", 1);
            return;
        }

        selector = trim_ascii(selector);
        if ((strcmp(selector, "left") == 0) || (strcmp(selector, "l") == 0)) {
            char *rest = saveptr;
            float values[4];
            unsigned count = 0u;

            if (rest == NULL) {
                emit_auto_pid_line("left", 0);
                return;
            }
            if (parse_pid_values(trim_ascii(rest), values, &count)) {
                apply_pid_values_to_wheel(0, values, count);
                uart_printf("auto,ack,pid,left\r\n");
                emit_auto_pid_line("left", 0);
            } else {
                uart_printf("auto,err,pid,left\r\n");
            }
            return;
        }
        if ((strcmp(selector, "right") == 0) || (strcmp(selector, "r") == 0)) {
            char *rest = saveptr;
            float values[4];
            unsigned count = 0u;

            if (rest == NULL) {
                emit_auto_pid_line("right", 1);
                return;
            }
            if (parse_pid_values(trim_ascii(rest), values, &count)) {
                apply_pid_values_to_wheel(1, values, count);
                uart_printf("auto,ack,pid,right\r\n");
                emit_auto_pid_line("right", 1);
            } else {
                uart_printf("auto,err,pid,right\r\n");
            }
            return;
        }
        if ((strcmp(selector, "both") == 0) || (strcmp(selector, "all") == 0)) {
            char *rest = saveptr;
            float values[4];
            unsigned count = 0u;

            if (rest == NULL) {
                emit_auto_pid_line("left", 0);
                emit_auto_pid_line("right", 1);
                return;
            }
            if (parse_pid_values(trim_ascii(rest), values, &count)) {
                apply_pid_values_to_wheel(0, values, count);
                apply_pid_values_to_wheel(1, values, count);
                uart_printf("auto,ack,pid,both\r\n");
                emit_auto_pid_line("left", 0);
                emit_auto_pid_line("right", 1);
            } else {
                uart_printf("auto,err,pid,both\r\n");
            }
            return;
        }

        uart_printf("auto,err,pid,selector\r\n");
        return;
    }

    uart_printf("auto,err,unknown,%s\r\n", token);
}

static void handle_command_line(test_task_ctx_t *ctx, char *line)
{
    float left_value;
    float right_value;

    line = trim_ascii(line);
    if (*line == '\0') {
        return;
    }

    if (strcmp(line, "help") == 0) {
        print_help();
        return;
    }
    if (strcmp(line, "showpid") == 0) {
        print_pid_line("left", 0);
        print_pid_line("right", 1);
        print_line_config();
        return;
    }
    if (strcmp(line, "lineraw") == 0) {
        print_line_raw_status();
        return;
    }
    if (strcmp(line, "linecfg") == 0) {
        print_line_config();
        return;
    }
    if (strcmp(line, "linecfg,reset") == 0) {
        LineTracker_ResetSensorMapping();
        print_line_config();
        return;
    }
    if (strncmp(line, "linecfg,", 8) == 0) {
        char *cursor = line + 8;
        char *endptr;
        long index;
        char *pin_name;

        index = strtol(cursor, &endptr, 10);
        if ((endptr == cursor) || (*endptr != ',')) {
            uart_printf("linecfg parse error: %s\r\n", line);
            return;
        }

        pin_name = trim_ascii(endptr + 1);
        if ((index < 0L) || (index >= 7L) || (*pin_name == '\0')) {
            uart_printf("linecfg parse error: %s\r\n", line);
            return;
        }

        if (!LineTracker_SetSensorPinByName((uint8_t)index, pin_name)) {
            uart_printf("linecfg invalid pin: %s\r\n", pin_name);
            return;
        }

        print_line_config();
        return;
    }
    if (strcmp(line, "stop") == 0) {
        apply_stop_command();
        uart_printf("cmd=stop\r\n");
        return;
    }
    if (strcmp(line, "imu") == 0) {
        s_imu_only_mode = (s_imu_only_mode == 0u) ? 1u : 0u;
        uart_printf("imu=%s\r\n", (s_imu_only_mode != 0u) ? "on" : "off");
        return;
    }
    if (strncmp(line, "imuz,", 5) == 0) {
        char *cursor = line + 5;
        char *endptr;
        float sign = strtof(cursor, &endptr);
        if (endptr != cursor) {
            Imu_SetGyroZSign(chassis_system_get_imu(), sign);
            uart_printf("imuz=%.1f gsens=%.1f\r\n",
                        chassis_system_get_imu()->cfg.gyro_z_sign,
                        chassis_system_get_imu()->gyro_sens_lsb_per_dps);
        }
        return;
    }
    if (strncmp(line, "imus,", 5) == 0) {
        char *cursor = line + 5;
        char *endptr;
        float sens = strtof(cursor, &endptr);
        if (endptr != cursor) {
            Imu_SetGyroSensOverride(chassis_system_get_imu(), sens);
            uart_printf("imus=%.1f gsens=%.1f\r\n",
                        chassis_system_get_imu()->cfg.gyro_sens_override,
                        chassis_system_get_imu()->gyro_sens_lsb_per_dps);
        }
        return;
    }
    if (strcmp(line, "q1") == 0) {
        select_challenge(APP_CHALLENGE_Q1);
        uart_printf("challenge=%s\r\n", app_challenge_name(APP_CHALLENGE_Q1));
        return;
    }
    if (strcmp(line, "q2") == 0) {
        select_challenge(APP_CHALLENGE_Q2);
        uart_printf("challenge=%s\r\n", app_challenge_name(APP_CHALLENGE_Q2));
        return;
    }
    if (strcmp(line, "q3") == 0) {
        select_challenge(APP_CHALLENGE_Q3);
        uart_printf("challenge=%s\r\n", app_challenge_name(APP_CHALLENGE_Q3));
        return;
    }
    if (strcmp(line, "q4") == 0) {
        select_challenge(APP_CHALLENGE_Q4);
        uart_printf("challenge=%s\r\n", app_challenge_name(APP_CHALLENGE_Q4));
        return;
    }
    if ((strcmp(line, "run") == 0) || (strcmp(line, "main") == 0)) {
        if (start_selected_challenge()) {
            app_challenge_info_t challenge;
            app_state_get_challenge(&challenge);
            uart_printf("cmd=run %s\r\n", app_challenge_name(challenge.active));
        } else {
            app_challenge_info_t challenge;
            app_state_get_challenge(&challenge);
            uart_printf("run blocked sel=%s\r\n",
                        app_challenge_name(challenge.selected));
        }
        return;
    }
    if (strncmp(line, "linepol,", 8) == 0) {
        char *endptr;
        long value = strtol(line + 8, &endptr, 10);

        while (isspace((unsigned char)*endptr)) {
            ++endptr;
        }
        if ((*endptr == '\0') && ((value == 0L) || (value == 1L))) {
            LineTracker_SetSensorLogic((value == 0L)
                                           ? LINE_SENSOR_BLACK_LOW
                                           : LINE_SENSOR_BLACK_HIGH);
            print_line_config();
        } else {
            uart_printf("line logic parse error: %s\r\n", line);
        }
        return;
    }
    if ((strncmp(line, "spd,", 4) == 0) || (strncmp(line, "speed,", 6) == 0)) {
        char *payload = (line[1] == 'p') ? line + 4 : line + 6;
        if (parse_float_pair(payload, &left_value, &right_value)) {
            apply_speed_command(left_value, right_value);
            uart_printf("cmd_spd=(%.3f,%.3f)\r\n", left_value, right_value);
        } else {
            uart_printf("cmd parse error: %s\r\n", line);
        }
        return;
    }
    if (strncmp(line, "twist,", 6) == 0) {
        if (parse_float_pair(line + 6, &left_value, &right_value)) {
            apply_twist_command(left_value, right_value);
            uart_printf("cmd_twist=(%.3f,%.3f)\r\n", left_value, right_value);
        } else {
            uart_printf("cmd parse error: %s\r\n", line);
        }
        return;
    }
    if (strncmp(line, "pidl,", 5) == 0) {
        float values[4];
        unsigned count = 0u;

        if (parse_pid_values(line + 5, values, &count)) {
            apply_pid_values_to_wheel(0, values, count);
            print_pid_line("left", 0);
        } else {
            uart_printf("pid parse error: %s\r\n", line);
        }
        return;
    }
    if (strncmp(line, "pidr,", 5) == 0) {
        float values[4];
        unsigned count = 0u;

        if (parse_pid_values(line + 5, values, &count)) {
            apply_pid_values_to_wheel(1, values, count);
            print_pid_line("right", 1);
        } else {
            uart_printf("pid parse error: %s\r\n", line);
        }
        return;
    }
    if (strncmp(line, "pid,", 4) == 0) {
        float values[4];
        unsigned count = 0u;

        if (parse_pid_values(line + 4, values, &count)) {
            apply_pid_values_to_wheel(0, values, count);
            apply_pid_values_to_wheel(1, values, count);
            print_pid_line("left", 0);
            print_pid_line("right", 1);
        } else {
            uart_printf("pid parse error: %s\r\n", line);
        }
        return;
    }
    if (strncmp(line, "auto,", 5) == 0) {
        handle_auto_command(ctx, line + 5);
        return;
    }
    if (parse_float_pair(line, &left_value, &right_value)) {
        apply_wheel_duty_command(left_value, right_value);
        uart_printf("cmd=(%.3f,%.3f)\r\n",
                    clampf(left_value * MODE_DUTY_SCALE, -1.0f, 1.0f),
                    clampf(right_value * MODE_DUTY_SCALE, -1.0f, 1.0f));
        return;
    }

    uart_printf("cmd parse error: %s\r\n", line);
}

static void poll_uart(test_task_ctx_t *ctx)
{
    char ch;

    while (uart_rx_get_char(&ch) != 0) {
        if ((ch == '\r') || (ch == '\n')) {
            if (ctx->rx_len > 0u) {
                ctx->rx_line[ctx->rx_len] = '\0';
                handle_command_line(ctx, ctx->rx_line);
                ctx->rx_len = 0u;
            }
            continue;
        }

        if (ctx->rx_len + 1u >= sizeof(ctx->rx_line)) {
            ctx->rx_len = 0u;
            uart_printf("cmd parse error: line too long\r\n");
            continue;
        }

        ctx->rx_line[ctx->rx_len++] = ch;
    }
}

static void poll_bt_uart(test_task_ctx_t *ctx)
{
    char ch;

    while (bt_uart_get_char(&ch) != 0) {
        if ((ch == '\r') || (ch == '\n')) {
            if (ctx->bt_rx_len > 0u) {
                ctx->bt_rx_line[ctx->bt_rx_len] = '\0';
                handle_command_line(ctx, ctx->bt_rx_line);
                ctx->bt_rx_len = 0u;
            }
            continue;
        }

        if (ctx->bt_rx_len + 1u >= sizeof(ctx->bt_rx_line)) {
            ctx->bt_rx_len = 0u;
            continue;
        }

        ctx->bt_rx_line[ctx->bt_rx_len++] = ch;
    }
}

void test_task(void *arg)
{
    TickType_t next = xTaskGetTickCount();
    test_task_ctx_t ctx;
    uint32_t now_ms;
    uint32_t irq_per_s;

    (void)arg;

    memset(&ctx, 0, sizeof(ctx));
    sanitize_phase_label(ctx.auto_phase, sizeof(ctx.auto_phase), "idle");
    ctx.last_report_tick_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    ctx.last_irq_count = EncoderHal_GetGpioIrqCount();

    uart_rx_init();
    bt_uart_init();
    apply_stop_command();
    print_help();

    for (;;) {
        poll_keys(&ctx);
        poll_uart(&ctx);
        poll_bt_uart(&ctx);
        emit_event_if_changed(&ctx);

        now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        if ((now_ms - ctx.last_report_tick_ms) >= MODE_REPORT_PERIOD_MS) {
            irq_per_s = calc_irq_per_second(&ctx, now_ms);
            if (ctx.auto_stream_enabled != 0u) {
                emit_auto_sample(&ctx, now_ms, irq_per_s);
            } else {
                emit_human_status(irq_per_s);
            }
        }

        vTaskDelayUntil(&next, pdMS_TO_TICKS(MODE_TASK_PERIOD_MS));
    }
}
