#include "pid.h"

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

static float normalize_alpha(float alpha)
{
    return clampf(alpha, 0.0f, 1.0f);
}

static void pid_reset_state(pid_state_t *state)
{
    state->integral = 0.0f;
    state->prev_error = 0.0f;
    state->prev_prev_error = 0.0f;
    state->prev_measurement = 0.0f;
    state->prev_output = 0.0f;
    state->d_term_filtered = 0.0f;
    state->last_output = 0.0f;
}

void Pid_Init(pid_controller_t *pid, const pid_config_t *config)
{
    if ((pid == NULL) || (config == NULL)) {
        return;
    }

    pid->config = *config;
    pid->config.d_filter_alpha = normalize_alpha(pid->config.d_filter_alpha);
    pid_reset_state(&pid->state);
}

void Pid_Reset(pid_controller_t *pid)
{
    if (pid == NULL) {
        return;
    }

    pid_reset_state(&pid->state);
}

void Pid_SetConfig(pid_controller_t *pid, const pid_config_t *config)
{
    if ((pid == NULL) || (config == NULL)) {
        return;
    }

    pid->config = *config;
    pid->config.d_filter_alpha = normalize_alpha(pid->config.d_filter_alpha);
}

void Pid_GetConfig(const pid_controller_t *pid, pid_config_t *config)
{
    if ((pid == NULL) || (config == NULL)) {
        return;
    }

    *config = pid->config;
}

void Pid_SetTunings(pid_controller_t *pid, float kp, float ki, float kd)
{
    if (pid == NULL) {
        return;
    }

    pid->config.kp = kp;
    pid->config.ki = ki;
    pid->config.kd = kd;
}

static float pid_compute_filtered_derivative(pid_controller_t *pid, float error, float fdb, float dt_s)
{
    float derivative_raw;
    float alpha;

    if (pid->config.mode == PID_MODE_INCREMENTAL) {
        derivative_raw = (error - 2.0f * pid->state.prev_error + pid->state.prev_prev_error) / dt_s;
    } else {
        derivative_raw = -(fdb - pid->state.prev_measurement) / dt_s;
    }

    alpha = pid->config.d_filter_alpha;
    pid->state.d_term_filtered =
        alpha * pid->state.d_term_filtered + (1.0f - alpha) * derivative_raw;
    return pid->state.d_term_filtered;
}

static float pid_select_integral(pid_controller_t *pid,
                                 float integral_candidate,
                                 float error,
                                 float unsat_output,
                                 float output)
{
    integral_candidate = clampf(integral_candidate,
                                pid->config.integral_min,
                                pid->config.integral_max);

    if (pid->config.anti_windup_enable == 0u) {
        return integral_candidate;
    }

    if (unsat_output == output) {
        return integral_candidate;
    }

    if (((unsat_output > pid->config.out_max) && (error < 0.0f)) ||
        ((unsat_output < pid->config.out_min) && (error > 0.0f))) {
        return integral_candidate;
    }

    return pid->state.integral;
}

float Pid_Update(pid_controller_t *pid, float ref, float fdb, float dt_s)
{
    float error;
    float derivative;
    float unsat_output;
    float output;

    if (pid == NULL) {
        return 0.0f;
    }

    if (dt_s <= 0.0f) {
        return pid->state.last_output;
    }

    error = ref - fdb;
    derivative = pid_compute_filtered_derivative(pid, error, fdb, dt_s);

    if (pid->config.mode == PID_MODE_INCREMENTAL) {
        float delta_output;
        float integral_candidate;

        integral_candidate = pid->state.integral + error * dt_s;

        delta_output =
            pid->config.kp * (error - pid->state.prev_error) +
            pid->config.ki * error * dt_s +
            pid->config.kd * derivative;
        unsat_output = pid->state.prev_output + delta_output;
        output = clampf(unsat_output, pid->config.out_min, pid->config.out_max);
        pid->state.integral = pid_select_integral(pid,
                                                  integral_candidate,
                                                  error,
                                                  unsat_output,
                                                  output);
    } else {
        float integral_candidate;

        integral_candidate = pid->state.integral + error * dt_s;
        unsat_output =
            pid->config.kp * error +
            pid->config.ki * integral_candidate +
            pid->config.kd * derivative;
        output = clampf(unsat_output, pid->config.out_min, pid->config.out_max);
        pid->state.integral = pid_select_integral(pid,
                                                  integral_candidate,
                                                  error,
                                                  unsat_output,
                                                  output);
    }

    if (pid->config.mode == PID_MODE_POSITIONAL) {
        unsat_output =
            pid->config.kp * error +
            pid->config.ki * pid->state.integral +
            pid->config.kd * derivative;
        output = clampf(unsat_output, pid->config.out_min, pid->config.out_max);
    }

    pid->state.prev_prev_error = pid->state.prev_error;
    pid->state.prev_error = error;
    pid->state.prev_measurement = fdb;
    pid->state.prev_output = output;
    pid->state.last_output = output;

    return output;
}
