#include "pid.h"

static inline float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void pid_init(pid_t *pid, const pid_params_t *params)
{
    pid->params               = *params;
    pid->setpoint             = 0.0f;
    pid->integral             = 0.0f;
    pid->last_measurement     = 0.0f;
    pid->last_error           = 0.0f;
    pid->last_deriv_filtered  = 0.0f;
    pid->primed               = false;
}

void pid_reset(pid_t *pid)
{
    pid->integral            = 0.0f;
    pid->last_measurement    = 0.0f;
    pid->last_error          = 0.0f;
    pid->last_deriv_filtered = 0.0f;
    pid->primed              = false;
}

void pid_set_params(pid_t *pid, const pid_params_t *params)
{
    pid->params = *params;
}

void pid_set_setpoint(pid_t *pid, float setpoint)
{
    pid->setpoint = setpoint;
}

float pid_step(pid_t *pid, float measurement, float dt)
{
    const pid_params_t *p = &pid->params;
    if (dt <= 0.0f) {
        /* 非法 dt 时保持上一次输出等价的结果，不更新任何状态。 */
        return clampf(p->kp * (pid->setpoint - measurement) + p->ki * pid->integral,
                      p->out_min, p->out_max);
    }

    const float error = pid->setpoint - measurement;

    /* ─── D 项 ─── */
    float deriv_raw;
    if (!pid->primed) {
        deriv_raw = 0.0f;  /* 首次无历史，避免虚假 kick */
    } else if (p->derivative_on_measurement) {
        /* 微分测量变化量（带负号，因为 e = sp - m, de = -dm when sp 恒定） */
        deriv_raw = -(measurement - pid->last_measurement) / dt;
    } else {
        deriv_raw = (error - pid->last_error) / dt;
    }

    /* 一阶 LPF：out = α * raw + (1-α) * last */
    const float alpha = p->deriv_lpf_alpha;
    const float deriv = pid->primed
        ? (alpha * deriv_raw + (1.0f - alpha) * pid->last_deriv_filtered)
        : deriv_raw;

    /* ─── 暂算不带积分更新的输出，用于判饱和 ─── */
    const float tentative = p->kp * error
                          + p->ki * pid->integral
                          + p->kd * deriv;

    const float saturated = clampf(tentative, p->out_min, p->out_max);

    /*
     * Back-calculation 反饱和：
     * 仅当 tentative 未饱和 或 误差方向与积分累加方向相反（可以解饱和）时，才累加积分。
     * 这样积分不会在硬限幅后继续"冲"。
     */
    const bool not_saturated = (tentative == saturated);
    const bool unwind_ok     = (saturated > tentative && error < 0.0f)
                            || (saturated < tentative && error > 0.0f);

    if (not_saturated || unwind_ok) {
        pid->integral = clampf(pid->integral + error * dt, p->int_min, p->int_max);
    }

    /* ─── 更新状态 ─── */
    pid->last_measurement    = measurement;
    pid->last_error          = error;
    pid->last_deriv_filtered = deriv;
    pid->primed              = true;

    return saturated;
}
