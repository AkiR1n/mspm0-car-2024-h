/*
 * LineTracker.h
 *
 * 基于MSPM0G3507的循迹传感器驱动程序
 * 使用TRM循迹传感器阵列，支持7路传感器输入
 */

#ifndef LINETRACKER_H
#define LINETRACKER_H

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// 循迹传感器数量
#define LINE_SENSOR_COUNT   7

// 传感器逻辑反转选项（如果传感器逻辑相反，将此值设为1）
#define SENSOR_LOGIC_INVERTED   1

// 传感器权重定义（用于计算偏差）
#define SENSOR_WEIGHT_0     -30     // 最左边传感器
#define SENSOR_WEIGHT_1     -20
#define SENSOR_WEIGHT_2     -10
#define SENSOR_WEIGHT_3     0       // 中间传感器
#define SENSOR_WEIGHT_4     10
#define SENSOR_WEIGHT_5     20
#define SENSOR_WEIGHT_6     30      // 最右边传感器

// 循迹状态枚举
typedef enum {
    LINE_STATE_ON_LINE = 0,         // 在线上
    LINE_STATE_LEFT_TURN,           // 需要左转
    LINE_STATE_RIGHT_TURN,          // 需要右转
    LINE_STATE_SHARP_LEFT,          // 急转左
    LINE_STATE_SHARP_RIGHT,         // 急转右
    LINE_STATE_NO_LINE,             // 没有检测到线
    LINE_STATE_ALL_LINE,            // 全部传感器检测到线（可能是起始线或结束线）
    LINE_STATE_UNKNOWN              // 未知状态
} LineState_t;

// 循迹传感器数据结构
typedef struct {
    uint8_t sensorValue[LINE_SENSOR_COUNT];  // 原始传感器值（0或1）
    uint8_t sensorBits;                      // 传感器位图表示
    int16_t linePosition;                    // 线的位置（-30到30）
    LineState_t lineState;                   // 当前线状态
    uint8_t activeSensorCount;               // 激活的传感器数量
    bool lineDetected;                       // 是否检测到线
} LineTracker_t;

// 全局变量
extern LineTracker_t g_lineTracker;

// 函数声明

/**
 * @brief 初始化循迹传感器
 */
void LineTracker_Init(void);

/**
 * @brief 读取所有传感器的值（用于中断中快速读取）
 */
void LineTracker_ReadSensors_Interrupt(void);

/**
 * @brief 读取所有传感器的值
 */
void LineTracker_ReadSensors(void);

/**
 * @brief 计算线的位置
 * @return 线的位置值（-30到30，0表示在中间）
 */
int16_t LineTracker_GetLinePosition(void);

/**
 * @brief 获取当前线状态
 * @return 线状态枚举值
 */
LineState_t LineTracker_GetLineState(void);

/**
 * @brief 获取传感器位图
 * @return 传感器状态的位图表示
 */
uint8_t LineTracker_GetSensorBits(void);

/**
 * @brief 检查是否检测到线
 * @return true表示检测到线，false表示没有检测到线
 */
bool LineTracker_IsLineDetected(void);

/**
 * @brief 获取激活的传感器数量
 * @return 当前激活的传感器数量
 */
uint8_t LineTracker_GetActiveSensorCount(void);

/**
 * @brief 获取指定传感器的值
 * @param sensorIndex 传感器索引（0-6）
 * @return 传感器值（0或1）
 */
uint8_t LineTracker_GetSensorValue(uint8_t sensorIndex);

/**
 * @brief 校准传感器（可选功能）
 */
void LineTracker_Calibrate(void);

#ifdef __cplusplus
}
#endif

#endif /* LINETRACKER_H */