#include "app_state.h"

volatile app_state_t g_app_state = {
    .imu_ready = 0u,
    .yaw_deg = 0.0f,
    .pitch_deg = 0.0f,
    .roll_deg = 0.0f,
    .line_position = 0,
    .line_bits = 0u,
    .left_pps = 0,
    .right_pps = 0,
    .mode = MOTION_MODE_STOP,
    .base_speed_pps = 0.0f,
    .last_turn = TURN_DIR_NONE,
};

const char *app_motion_mode_name(motion_mode_t mode)
{
    switch (mode) {
    case MOTION_MODE_LINE_FOLLOW:
        return "LINE";
    case MOTION_MODE_YAW_HOLD:
        return "YAW";
    case MOTION_MODE_DIFFERENTIAL:
        return "DIFF";
    case MOTION_MODE_OPEN_LOOP:
        return "OPEN";
    case MOTION_MODE_STOP:
    default:
        return "STOP";
    }
}

const char *app_turn_name(turn_dir_t dir)
{
    switch (dir) {
    case TURN_DIR_LEFT:
        return "LEFT";
    case TURN_DIR_RIGHT:
        return "RIGHT";
    case TURN_DIR_NONE:
    default:
        return "NONE";
    }
}
