#include "turn_detection.h"
#include "linetracker.h"
#include "clock.h"
#include "ti_msp_dl_config.h"

// 全局变量定义
Turn_Detection_t g_turnDetection;

/**
 * @brief 初始化转弯检测模块
 */
void TurnDetection_Init(void)
{
    // 清空数据结构
    g_turnDetection.state = TURN_STATE_IDLE;
    g_turnDetection.left_sensor_count = 0;
    g_turnDetection.detect_start_time = 0;
    g_turnDetection.inhibit_start_time = 0;
    g_turnDetection.last_turn_time = 0;
    g_turnDetection.turn_ready = false;
    
    // 启动TIMG8定时器用于转弯检测
    NVIC_EnableIRQ(TIMER_TRACKER_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_TRACKER_INST);
}

/**
 * @brief 更新转弯检测状态（应在定时器中断或主循环中定期调用）
 */
void TurnDetection_Update(void)
{
    // 更新传感器数据
    LineTracker_ReadSensors_Interrupt();
    
    // 检查是否在转弯抑制期内
    bool turn_inhibited = (tick_ms - g_turnDetection.last_turn_time) < TURN_INHIBIT_TIME_MS;
    if (turn_inhibited && g_turnDetection.state != TURN_STATE_INHIBITED) {
        g_turnDetection.state = TURN_STATE_INHIBITED;
        g_turnDetection.inhibit_start_time = tick_ms;
    } else if (!turn_inhibited && g_turnDetection.state == TURN_STATE_INHIBITED) {
        g_turnDetection.state = TURN_STATE_IDLE;
    }
    
    // 检查左侧转弯条件：左侧3路传感器中有≥2路检测到线
    uint8_t left_sensor_count = 0;
    if (g_lineTracker.sensorValue[0]) left_sensor_count++; // 最左
    if (g_lineTracker.sensorValue[1]) left_sensor_count++; // 左
    if (g_lineTracker.sensorValue[2]) left_sensor_count++; // 中左
    
    g_turnDetection.left_sensor_count = left_sensor_count;
    bool turn_condition = (left_sensor_count >= TURN_DETECT_COUNT_MIN) && !turn_inhibited;
    
    switch (g_turnDetection.state) {
        case TURN_STATE_IDLE:
            if (turn_condition) {
                // 检测到转弯信号
                g_turnDetection.state = TURN_STATE_DETECTED;
                g_turnDetection.detect_start_time = tick_ms;
            }
            break;
            
        case TURN_STATE_DETECTED:
            if (!turn_condition) {
                // 转弯信号消失，回到空闲状态
                g_turnDetection.state = TURN_STATE_IDLE;
            } else {
                // 检查转弯信号是否稳定
                uint32_t detect_duration = tick_ms - g_turnDetection.detect_start_time;
                bool quick_confirm = (left_sensor_count >= 2);
                bool stable_confirm = (detect_duration >= TURN_DETECT_STABLE_MS);
                
                // 如果转弯信号稳定，确认转弯
                if (quick_confirm || stable_confirm) {
                    g_turnDetection.state = TURN_STATE_CONFIRMED;
                    g_turnDetection.turn_ready = true;
                } else if (detect_duration >= TURN_DETECT_TIMEOUT_MS) {
                    // 检测超时，回到空闲状态
                    g_turnDetection.state = TURN_STATE_IDLE;
                }
            }
            break;
            
        case TURN_STATE_CONFIRMED:
            // 等待转弯被处理
            break;
            
        case TURN_STATE_INHIBITED:
            // 检查抑制是否结束
            if ((tick_ms - g_turnDetection.inhibit_start_time) >= TURN_INHIBIT_TIME_MS) {
                g_turnDetection.state = TURN_STATE_IDLE;
            }
            break;
    }
}

/**
 * @brief 检查是否准备好转弯
 * @return true表示准备好转弯，false表示未准备好
 */
bool TurnDetection_IsTurnReady(void)
{
    return g_turnDetection.turn_ready;
}

/**
 * @brief 重置转弯检测状态（在完成转弯后调用）
 */
void TurnDetection_Reset(void)
{
    g_turnDetection.state = TURN_STATE_INHIBITED;
    g_turnDetection.turn_ready = false;
    g_turnDetection.last_turn_time = tick_ms;
    g_turnDetection.inhibit_start_time = tick_ms;
}