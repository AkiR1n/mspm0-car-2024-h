#ifndef APP_STATE_H_
#define APP_STATE_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float    dt_s;
    float    left_speed_mps;
    float    right_speed_mps;
    int32_t  left_count;
    int32_t  right_count;
    uint8_t  imu_ready;
    uint8_t  imu_stable;
    float    yaw_deg;
    float    gyro_z;
    float    pitch_deg;
    float    roll_deg;
    float    accel_x;
    float    accel_y;
    float    accel_z;
    float    gyro_x;
    float    gyro_y;
    uint32_t imu_uptime_ms;
    uint32_t imu_stable_ms;
    uint8_t  line_bits;
    uint8_t  line_detected;
    int16_t  line_position;
} chassis_feedback_t;

typedef struct {
    uint8_t stop;
    uint8_t enable_closed_loop;
    float   v_mps;
    float   w_radps;
    float   left_duty;
    float   right_duty;
    float   left_speed_mps;
    float   right_speed_mps;
} chassis_command_t;

typedef struct {
    float left_target_mps;
    float right_target_mps;
    float left_measured_mps;
    float right_measured_mps;
    float left_motor_duty;
    float right_motor_duty;
} chassis_debug_t;

typedef enum {
    APP_MODE_STOP = 0,
    APP_MODE_TWIST_OPEN = 1,
    APP_MODE_WHEEL_TEST = 2,
    APP_MODE_WHEEL_SPEED_TEST = 3,
    APP_MODE_MAIN = 4,
    APP_MODE_STRAIGHT_TEST = 5,
    APP_MODE_LINE_TEST = 6,
} app_mode_t;

typedef enum {
    APP_MAIN_STATE_IDLE = 0,
    APP_MAIN_STATE_ALIGN,
    APP_MAIN_STATE_GAP,
    APP_MAIN_STATE_ARC_TRACK,
    APP_MAIN_STATE_ARC_LOST_LEFT,
    APP_MAIN_STATE_ARC_LOST_RIGHT,
    APP_MAIN_STATE_STOPPED,
} app_main_state_t;

typedef enum {
    APP_CHALLENGE_NONE = 0,
    APP_CHALLENGE_Q1 = 1,
    APP_CHALLENGE_Q2 = 2,
    APP_CHALLENGE_Q3 = 3,
    APP_CHALLENGE_Q4 = 4,
} app_challenge_t;

typedef enum {
    APP_CHALLENGE_STATUS_READY = 0,
    APP_CHALLENGE_STATUS_ALIGN,
    APP_CHALLENGE_STATUS_RUNNING,
    APP_CHALLENGE_STATUS_DONE,
} app_challenge_status_t;

typedef enum {
    APP_PHASE_ACTION_NONE = 0,
    APP_PHASE_ACTION_ALIGN_START,
    APP_PHASE_ACTION_GAP_TRAVERSE,
    APP_PHASE_ACTION_ARC_TRACK,
    APP_PHASE_ACTION_STOP_AND_SIGNAL,
} app_phase_action_t;

typedef enum {
    APP_CHALLENGE_PHASE_IDLE = 0,
    APP_CHALLENGE_PHASE_ALIGN_A_TO_B,
    APP_CHALLENGE_PHASE_ALIGN_A_TO_C,
    APP_CHALLENGE_PHASE_GAP_AB,
    APP_CHALLENGE_PHASE_ARC_BC,
    APP_CHALLENGE_PHASE_GAP_CD,
    APP_CHALLENGE_PHASE_ARC_DA,
    APP_CHALLENGE_PHASE_GAP_AC,
    APP_CHALLENGE_PHASE_ARC_CB,
    APP_CHALLENGE_PHASE_GAP_BD,
    APP_CHALLENGE_PHASE_STOP_A,
    APP_CHALLENGE_PHASE_STOP_B,
} app_challenge_phase_t;

typedef enum {
    APP_EVENT_NONE = 0,
    APP_EVENT_START,
    APP_EVENT_PASS_A,
    APP_EVENT_PASS_B,
    APP_EVENT_PASS_C,
    APP_EVENT_PASS_D,
    APP_EVENT_STOP,
} app_event_id_t;

typedef struct {
    app_challenge_t        selected;
    app_challenge_t        active;
    app_challenge_status_t status;
    app_challenge_phase_t  phase;
    app_phase_action_t     action;
    uint8_t                checkpoint_count;
    uint8_t                lap_index;
    uint8_t                lap_total;
    float                  geometry_heading_deg;
    float                  hold_heading_deg;
    float                  heading_error_deg;
    float                  phase_distance_m;
    float                  target_distance_m;
    float                  target_speed_mps;
    app_event_id_t         last_event;
    uint32_t               last_event_ms;
    uint32_t               event_seq;
} app_challenge_info_t;

typedef struct {
    app_mode_t           mode;
    app_main_state_t     main_state;
    app_challenge_info_t challenge;
    chassis_feedback_t   feedback;
    chassis_command_t    command;
    chassis_debug_t      debug;
} app_state_snapshot_t;

void app_state_init(void);
void app_state_set_mode(app_mode_t mode);
app_mode_t app_state_get_mode(void);
void app_state_set_main_state(app_main_state_t state);
app_main_state_t app_state_get_main_state(void);
void app_state_set_challenge(const app_challenge_info_t *challenge);
void app_state_get_challenge(app_challenge_info_t *challenge);
void app_state_emit_event(app_event_id_t event_id);
void app_state_set_feedback(const chassis_feedback_t *feedback);
void app_state_get_feedback(chassis_feedback_t *feedback);
void app_state_set_command(const chassis_command_t *command);
void app_state_get_command(chassis_command_t *command);
void app_state_set_debug(const chassis_debug_t *debug);
void app_state_get_debug(chassis_debug_t *debug);
void app_state_get_snapshot(app_state_snapshot_t *snapshot);
uint8_t app_challenge_lap_total(app_challenge_t challenge);
const char *app_mode_name(app_mode_t mode);
const char *app_main_state_name(app_main_state_t state);
const char *app_challenge_name(app_challenge_t challenge);
const char *app_challenge_status_name(app_challenge_status_t status);
const char *app_phase_action_name(app_phase_action_t action);
const char *app_challenge_phase_name(app_challenge_phase_t phase);
const char *app_event_name(app_event_id_t event_id);

#endif
