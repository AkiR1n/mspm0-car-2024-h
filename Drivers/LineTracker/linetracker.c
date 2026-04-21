/*
 * LineTracker.c
 *
 * 基于MSPM0G3507的循迹传感器驱动程序实现
 * 使用TRM循迹传感器阵列，支持7路传感器输入
 */

#include "linetracker.h"
#include <stdio.h>
#include <string.h>

// 全局变量定义
LineTracker_t g_lineTracker;

// 传感器权重数组（用于位置计算）
static const int16_t sensorWeights[LINE_SENSOR_COUNT] = {
    SENSOR_WEIGHT_0, SENSOR_WEIGHT_1, SENSOR_WEIGHT_2, SENSOR_WEIGHT_3,
    SENSOR_WEIGHT_4, SENSOR_WEIGHT_5, SENSOR_WEIGHT_6
};

// 传感器GPIO端口和引脚配置
static const struct {
    uint32_t port;
    uint32_t pin;
} sensorPins[LINE_SENSOR_COUNT] = {
    {(uint32_t)GPIO_TRM_PIN_OUT1_PORT, GPIO_TRM_PIN_OUT1_PIN},  // 传感器0 - 最左边
    {(uint32_t)GPIO_TRM_PIN_OUT2_PORT, GPIO_TRM_PIN_OUT2_PIN},  // 传感器1
    {(uint32_t)GPIO_TRM_PIN_OUT3_PORT, GPIO_TRM_PIN_OUT3_PIN},  // 传感器2
    {(uint32_t)GPIO_TRM_PIN_OUT4_PORT, GPIO_TRM_PIN_OUT4_PIN},  // 传感器3 - 中间
    {(uint32_t)GPIO_TRM_PIN_OUT5_PORT, GPIO_TRM_PIN_OUT5_PIN},  // 传感器4
    {(uint32_t)GPIO_TRM_PIN_OUT6_PORT, GPIO_TRM_PIN_OUT6_PIN},  // 传感器5
    {(uint32_t)GPIO_TRM_PIN_OUT7_PORT, GPIO_TRM_PIN_OUT7_PIN}   // 传感器6 - 最右边
};

/**
 * @brief 初始化循迹传感器
 */
void LineTracker_Init(void)
{
    // 清空数据结构
    memset(&g_lineTracker, 0, sizeof(LineTracker_t));
    
    // GPIO已经在ti_msp_dl_config.c中配置，这里只需要确保引脚已经初始化
    // 传感器引脚配置为输入模式，在SysConfig中已经完成
    
    printf("LineTracker: 初始化完成，7路传感器已配置\n");
}

/**
 * @brief 读取所有传感器的值（用于中断中快速读取）
 */
void LineTracker_ReadSensors_Interrupt(void)
{
    uint8_t i;
    
    // 清空之前的状态
    g_lineTracker.sensorBits = 0;
    g_lineTracker.activeSensorCount = 0;
    
    // 读取每个传感器的值
    for (i = 0; i < LINE_SENSOR_COUNT; i++) {
        // 读取GPIO引脚状态
        uint32_t pinState = DL_GPIO_readPins((GPIO_Regs *)sensorPins[i].port, sensorPins[i].pin);
        
        // 根据配置决定传感器逻辑
        #if SENSOR_LOGIC_INVERTED
            // 反向逻辑：低电平=白色背景，高电平=检测到黑线
            g_lineTracker.sensorValue[i] = (pinState != 0) ? 1 : 0;
        #else
            // 正常逻辑：高电平=白色背景，低电平=检测到黑线
            g_lineTracker.sensorValue[i] = (pinState != 0) ? 0 : 1;
        #endif
        
        // 更新位图
        if (g_lineTracker.sensorValue[i]) {
            g_lineTracker.sensorBits |= (1 << i);
            g_lineTracker.activeSensorCount++;
        }
    }
}

/**
 * @brief 读取所有传感器的值
 */
void LineTracker_ReadSensors(void)
{
    // 使用中断版本的读取函数
    LineTracker_ReadSensors_Interrupt();
    
    // 计算线位置
    g_lineTracker.linePosition = LineTracker_GetLinePosition();
    
    // 确定线状态
    g_lineTracker.lineState = LineTracker_GetLineState();
    
    // 更新线检测状态
    g_lineTracker.lineDetected = (g_lineTracker.activeSensorCount > 0);
}

/**
 * @brief 计算线的位置
 * @return 线的位置值（-30到30，0表示在中间）
 */
int16_t LineTracker_GetLinePosition(void)
{
    int32_t weightedSum = 0;
    int32_t totalWeight = 0;
    uint8_t i;
    
    // 如果没有传感器激活，返回上次的位置
    if (g_lineTracker.activeSensorCount == 0) {
        return g_lineTracker.linePosition;
    }
    
    // 计算加权平均位置
    for (i = 0; i < LINE_SENSOR_COUNT; i++) {
        if (g_lineTracker.sensorValue[i]) {
            weightedSum += sensorWeights[i];
            totalWeight += 1;
        }
    }
    
    if (totalWeight > 0) {
        return (int16_t)(weightedSum / totalWeight);
    }
    
    return 0;
}

/**
 * @brief 获取当前线状态
 * @return 线状态枚举值
 */
LineState_t LineTracker_GetLineState(void)
{
    uint8_t bits = g_lineTracker.sensorBits;
    uint8_t count = g_lineTracker.activeSensorCount;
    
    // 没有检测到线
    if (count == 0) {
        return LINE_STATE_NO_LINE;
    }
    
    // 所有传感器都检测到线（可能是起始线或十字路口）
    if (count == LINE_SENSOR_COUNT) {
        return LINE_STATE_ALL_LINE;
    }
    
    // 根据传感器模式判断状态
    switch (bits) {
        // 中间传感器激活 - 在线上
        case 0b0001000:  // 只有中间传感器
        case 0b0011000:  // 中间和左中
        case 0b0001100:  // 中间和右中
        case 0b0011100:  // 中间三个
            return LINE_STATE_ON_LINE;
            
        // 左转情况
        case 0b0000001:  // 最左边
        case 0b0000011:  // 左边两个
        case 0b0000111:  // 左边三个
        case 0b0110000:  // 左中两个
        case 0b0111000:  // 左中三个
            return LINE_STATE_LEFT_TURN;
            
        // 右转情况
        case 0b1000000:  // 最右边
        case 0b1100000:  // 右边两个
        case 0b1110000:  // 右边三个
        case 0b0000110:  // 右中两个
        case 0b0001110:  // 右中三个
            return LINE_STATE_RIGHT_TURN;
            
        // 急转弯情况 - 只有边缘传感器激活
        case 0b0000010:  // 只有左二
        case 0b0000100:  // 只有左三
            return LINE_STATE_SHARP_LEFT;
            
        case 0b0100000:  // 只有右二
        case 0b0010000:  // 只有右三
            return LINE_STATE_SHARP_RIGHT;
            
        default:
            // 根据位置判断
            if (g_lineTracker.linePosition < -15) {
                return LINE_STATE_LEFT_TURN;
            } else if (g_lineTracker.linePosition > 15) {
                return LINE_STATE_RIGHT_TURN;
            } else {
                return LINE_STATE_ON_LINE;
            }
    }
}

/**
 * @brief 获取传感器位图
 * @return 传感器状态的位图表示
 */
uint8_t LineTracker_GetSensorBits(void)
{
    return g_lineTracker.sensorBits;
}

/**
 * @brief 检查是否检测到线
 * @return true表示检测到线，false表示没有检测到线
 */
bool LineTracker_IsLineDetected(void)
{
    return g_lineTracker.lineDetected;
}

/**
 * @brief 获取激活的传感器数量
 * @return 当前激活的传感器数量
 */
uint8_t LineTracker_GetActiveSensorCount(void)
{
    return g_lineTracker.activeSensorCount;
}

/**
 * @brief 获取指定传感器的值
 * @param sensorIndex 传感器索引（0-6）
 * @return 传感器值（0或1）
 */
uint8_t LineTracker_GetSensorValue(uint8_t sensorIndex)
{
    if (sensorIndex < LINE_SENSOR_COUNT) {
        return g_lineTracker.sensorValue[sensorIndex];
    }
    return 0;
}

/**
 * @brief 校准传感器（可选功能）
 */
void LineTracker_Calibrate(void)
{
    // 这里可以实现传感器校准功能
    // 例如：记录传感器在白色和黑色表面的阈值
    printf("LineTracker: 校准功能预留，当前使用默认阈值\n");
}

/* ======================== 调试功能（已注释） ======================== */

/*
 * @brief 打印传感器状态（用于调试）
 */
/*
void LineTracker_PrintStatus(void)
{
    uint8_t i;
    
    printf("LineTracker状态:\n");
    printf("传感器值: ");
    for (i = 0; i < LINE_SENSOR_COUNT; i++) {
        printf("%d ", g_lineTracker.sensorValue[i]);
    }
    printf("\n");
    
    printf("位图: 0b");
    for (i = 0; i < LINE_SENSOR_COUNT; i++) {
        printf("%d", (g_lineTracker.sensorBits >> (LINE_SENSOR_COUNT - 1 - i)) & 1);
    }
    printf(" (0x%02X)\n", g_lineTracker.sensorBits);
    
    printf("线位置: %d\n", g_lineTracker.linePosition);
    printf("激活传感器数: %d\n", g_lineTracker.activeSensorCount);
    
    const char* stateNames[] = {
        "在线上", "左转", "右转", "急转左", "急转右", 
        "无线", "全线", "未知"
    };
    printf("线状态: %s\n", stateNames[g_lineTracker.lineState]);
    printf("检测到线: %s\n", g_lineTracker.lineDetected ? "是" : "否");
    printf("------------------------\n");
}
*/

/*
 * @brief GPIO调试函数 - 检查GPIO读取是否正常
 */
/*
void LineTracker_DebugGPIO(void)
{
    uint8_t i;
    uint32_t rawValue;
    
    printf("=== GPIO调试信息 ===\n");
    
    for (i = 0; i < LINE_SENSOR_COUNT; i++) {
        // 读取原始GPIO值
        rawValue = DL_GPIO_readPins((GPIO_Regs *)sensorPins[i].port, sensorPins[i].pin);
        
        printf("传感器 %d: 端口=0x%08X, 引脚=0x%08X, 原始值=%lu, 处理后=%d\n", 
               i, 
               (uint32_t)sensorPins[i].port, 
               sensorPins[i].pin, 
               rawValue, 
               rawValue ? 0 : 1);
    }
    
    printf("==================\n");
}
*/