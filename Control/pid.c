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

void Pid_Init(pid_t *pid,
              float kp,
              float ki,
              float kd,
              float out_min,
              float out_max,
              float integral_min,
              float integral_max)
{
    if (pid == NULL) {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->integral_min = integral_min;
    pid->integral_max = integral_max;
}

void Pid_Reset(pid_t *pid)
{
    if (pid == NULL) {
        return;
    }

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

float Pid_Update(pid_t *pid, float ref, float fdb, float dt_s)
{
    float error;
    float derivative;
    float output;

    if ((pid == NULL) || (dt_s <= 0.0f)) {
        return 0.0f;
    }

    error = ref - fdb;
    pid->integral += error * dt_s;
    pid->integral = clampf(pid->integral, pid->integral_min, pid->integral_max);
    derivative = (error - pid->prev_error) / dt_s;
    pid->prev_error = error;

    output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    return clampf(output, pid->out_min, pid->out_max);
}
