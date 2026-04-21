#ifndef APP_STATE_H_
#define APP_STATE_H_

#include <stdint.h>

#include "motion_control.h"
#include "turn_detection.h"

typedef struct {
    uint8_t       imu_ready;
    float         yaw_deg;
    float         pitch_deg;
    float         roll_deg;
    int16_t       line_position;
    uint8_t       line_bits;
    int32_t       left_pps;
    int32_t       right_pps;
    motion_mode_t mode;
    float         base_speed_pps;
    turn_dir_t    last_turn;
} app_state_t;

extern volatile app_state_t g_app_state;

const char *app_motion_mode_name(motion_mode_t mode);
const char *app_turn_name(turn_dir_t dir);

#endif
