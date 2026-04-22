#ifndef CONTROL_PID_H_
#define CONTROL_PID_H_

#include <stddef.h>

typedef enum {
    PID_MODE_POSITIONAL = 0,
    PID_MODE_INCREMENTAL = 1,
} pid_mode_t;

typedef struct {
    float      kp;
    float      ki;
    float      kd;
    float      out_min;
    float      out_max;
    float      integral_min;
    float      integral_max;
    float      d_filter_alpha;
    unsigned   anti_windup_enable : 1;
    pid_mode_t mode;
} pid_config_t;

typedef struct {
    float integral;
    float prev_error;
    float prev_prev_error;
    float prev_measurement;
    float prev_output;
    float d_term_filtered;
    float last_output;
} pid_state_t;

typedef struct {
    pid_config_t config;
    pid_state_t  state;
} pid_controller_t;

void Pid_Init(pid_controller_t *pid, const pid_config_t *config);
void Pid_Reset(pid_controller_t *pid);
void Pid_SetConfig(pid_controller_t *pid, const pid_config_t *config);
void Pid_GetConfig(const pid_controller_t *pid, pid_config_t *config);
void Pid_SetTunings(pid_controller_t *pid, float kp, float ki, float kd);
float Pid_Update(pid_controller_t *pid, float ref, float fdb, float dt_s);

#endif
