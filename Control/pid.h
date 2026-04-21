#ifndef CONTROL_PID_H_
#define CONTROL_PID_H_

#include <stddef.h>

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float out_min;
    float out_max;
    float integral_min;
    float integral_max;
} pid_t;

void Pid_Init(pid_t *pid,
              float kp,
              float ki,
              float kd,
              float out_min,
              float out_max,
              float integral_min,
              float integral_max);
void Pid_Reset(pid_t *pid);
float Pid_Update(pid_t *pid, float ref, float fdb, float dt_s);

#endif
