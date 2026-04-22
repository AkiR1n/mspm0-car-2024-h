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
} app_mode_t;

typedef enum {
    APP_MAIN_STATE_IDLE = 0,
    APP_MAIN_STATE_TRACK,
    APP_MAIN_STATE_LOST_LEFT,
    APP_MAIN_STATE_LOST_RIGHT,
} app_main_state_t;

typedef struct {
    app_mode_t          mode;
    app_main_state_t    main_state;
    chassis_feedback_t  feedback;
    chassis_command_t   command;
    chassis_debug_t     debug;
} app_state_snapshot_t;

void app_state_init(void);
void app_state_set_mode(app_mode_t mode);
app_mode_t app_state_get_mode(void);
void app_state_set_main_state(app_main_state_t state);
app_main_state_t app_state_get_main_state(void);
void app_state_set_feedback(const chassis_feedback_t *feedback);
void app_state_get_feedback(chassis_feedback_t *feedback);
void app_state_set_command(const chassis_command_t *command);
void app_state_get_command(chassis_command_t *command);
void app_state_set_debug(const chassis_debug_t *debug);
void app_state_get_debug(chassis_debug_t *debug);
void app_state_get_snapshot(app_state_snapshot_t *snapshot);
const char *app_mode_name(app_mode_t mode);
const char *app_main_state_name(app_main_state_t state);

#endif
