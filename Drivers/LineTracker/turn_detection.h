#ifndef TURN_DETECTION_H
#define TURN_DETECTION_H

#include <stdint.h>
#include <stdbool.h>

// 转弯检测参数
#define TURN_DETECT_COUNT_MIN 3       // 最少检测到的左侧传感器数量
#define TURN_DETECT_STABLE_MS 2       // 转弯信号稳定时间（毫秒）
#define TURN_DETECT_TIMEOUT_MS 100    // 转弯检测最大等待时间
#define TURN_INHIBIT_TIME_MS 800      // 转弯后的抑制时间（防止重复检测）

// 转弯检测状态
typedef enum {
    TURN_STATE_IDLE,           // 空闲状态
    TURN_STATE_DETECTED,       // 检测到转弯信号
    TURN_STATE_CONFIRMED,      // 转弯信号已确认
    TURN_STATE_INHIBITED       // 抑制状态
} Turn_State_t;

// 转弯检测结构体
typedef struct {
    Turn_State_t state;              // 当前状态
    uint8_t left_sensor_count;       // 左侧传感器检测数量
    uint32_t detect_start_time;      // 检测开始时间
    uint32_t inhibit_start_time;     // 抑制开始时间
    uint32_t last_turn_time;         // 上次转弯时间
    bool turn_ready;                 // 转弯准备就绪标志
} Turn_Detection_t;

// 全局变量声明
extern Turn_Detection_t g_turnDetection;

// 函数声明
void TurnDetection_Init(void);
void TurnDetection_Update(void);
bool TurnDetection_IsTurnReady(void);
void TurnDetection_Reset(void);

#endif /* TURN_DETECTION_H */