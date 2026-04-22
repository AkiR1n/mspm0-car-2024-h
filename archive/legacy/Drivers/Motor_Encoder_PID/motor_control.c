/*
 * motor_control.c
 *
 *  电机控制实现 - 集成多种控制模式
 *  
 *  支持的控制模式：
 *  1. MOTOR_MODE_LINE_FOLLOWING  - 7路循迹传感器PID控制
 *  2. MOTOR_MODE_YAW_CORRECTION  - MPU6050 Yaw角闭环控制  
 *  3. MOTOR_MODE_SPEED_CONTROL   - 直接速度控制（用于精确转弯）
 *  4. MOTOR_MODE_MANUAL          - 手动控制模式
 *  5. MOTOR_MODE_STOP            - 停止模式
 */

#include "motor_control.h"
#include "Encoder.h"
#include "linetracker.h"
#include <math.h>

// 循迹PID控制器参数设置
#define LINE_PID_KP         1.2f   // 比例增益
#define LINE_PID_KI         0.0f   // 积分增益
#define LINE_PID_KD         0.6f   // 微分增益
#define LINE_PID_INT_LIMIT  100.0f // 积分限幅
#define LINE_PID_OUT_LIMIT  20.0f  // 输出限幅

// Yaw角PID控制器参数设置
#define YAW_PID_KP          1.0f
#define YAW_PID_KI          0.0f
#define YAW_PID_KD          0.2f
#define YAW_PID_INT_LIMIT   200.0f
#define YAW_PID_OUT_LIMIT   100.0f

// 速度环PID控制器参数设置
#define SPEED_PID_KP        1.2f
#define SPEED_PID_KI        0.2f
#define SPEED_PID_KD        0.1f
#define SPEED_PID_INT_LIMIT 50.0f
#define SPEED_PID_OUT_LIMIT 100.0f

// 添加最大速度限制，防止电机跑满导致失控
#define MAX_MOTOR_SPEED 45.0f

// 添加电机平衡因子，用于补偿左右电机速度差异
#define MOTOR_BALANCE_FACTOR 1.0f

Motor_Control_t g_motorControl;

/**
 * @brief 初始化电机控制器
 * 
 * 该函数完成以下初始化工作：
 * 1. 初始化电机驱动模块
 * 2. 初始化编码器模块
 * 3. 初始化各个PID控制器（循迹、Yaw角控制、左右轮速度控制）
 * 4. 设置初始控制模式和参数
 */
void MotorControl_Init(void)
{
    // 初始化电机和编码器
    Motor_Init();
    Encoder_Init();
    LineTracker_Init();

    // 初始化PID控制器
    // 循迹PID控制器用于根据线位置偏差调整小车方向
    PID_Init(&g_motorControl.line_pid, LINE_PID_KP, LINE_PID_KI, LINE_PID_KD, LINE_PID_INT_LIMIT, LINE_PID_OUT_LIMIT);
    // Yaw角PID控制器用于保持或调整小车的朝向
    PID_Init(&g_motorControl.yaw_pid, YAW_PID_KP, YAW_PID_KI, YAW_PID_KD, YAW_PID_INT_LIMIT, YAW_PID_OUT_LIMIT);
    // 左轮速度PID控制器用于精确控制左轮转速
    PID_Init(&g_motorControl.speed_pid_L, SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, SPEED_PID_INT_LIMIT, SPEED_PID_OUT_LIMIT);
    // 右轮速度PID控制器用于精确控制右轮转速
    PID_Init(&g_motorControl.speed_pid_R, SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, SPEED_PID_INT_LIMIT, SPEED_PID_OUT_LIMIT);

    // 设置初始状态
    g_motorControl.mode = MOTOR_MODE_STOP;
    g_motorControl.base_speed = 20.0f;
    g_motorControl.target_yaw = 0.0f;
    g_motorControl.left_speed_target = 0.0f;
    g_motorControl.right_speed_target = 0.0f;
    
    // 设置循迹PID的目标值为0（保持在线中央）
    PID_SetTarget(&g_motorControl.line_pid, 0.0f);
}

/**
 * @brief 设置电机控制模式
 * @param mode 新的控制模式
 */
void MotorControl_SetMode(Motor_Mode_t mode)
{
    g_motorControl.mode = mode;
    if (mode == MOTOR_MODE_STOP) {
        MotorControl_Stop();
    }
}

/**
 * @brief 设置基础速度
 * @param speed 基础速度值 (0.0 - 50.0)
 */
void MotorControl_SetBaseSpeed(float speed)
{
    // 限制基础速度范围
    if (speed > MAX_MOTOR_SPEED) {
        speed = MAX_MOTOR_SPEED;
    } else if (speed < 0.0f) {
        speed = 0.0f;
    }
    g_motorControl.base_speed = speed;
}

/**
 * @brief 设置目标Yaw角
 * @param yaw 目标Yaw角
 */
void MotorControl_SetTargetYaw(float yaw)
{
    g_motorControl.target_yaw = yaw;
    PID_SetTarget(&g_motorControl.yaw_pid, yaw);
}

/**
 * @brief 设置左右轮目标速度（用于差速转弯等精确控制）
 * @param left_speed 左轮目标速度
 * @param right_speed 右轮目标速度
 */
void MotorControl_SetSpeedTarget(float left_speed, float right_speed)
{
    g_motorControl.left_speed_target = left_speed;
    g_motorControl.right_speed_target = right_speed;
}

/**
 * @brief 更新电机控制状态，应在主循环中定期调用
 * 
 * 这是电机控制的核心函数，根据当前控制模式计算并设置电机输出：
 * 1. 根据控制模式计算左右轮目标速度
 * 2. 使用编码器获取实际速度反馈
 * 3. 通过PID控制器计算速度控制输出
 * 4. 设置电机PWM驱动值
 */
void MotorControl_Update(void)
{
    float line_correction = 0;       // 循迹修正值
    float yaw_correction = 0;        // Yaw角修正值
    float left_speed_target = 0;    // 左轮目标速度
    float right_speed_target = 0;   // 右轮目标速度

    // 更新循迹传感器数据
    LineTracker_ReadSensors();
    
    // 根据控制模式计算目标速度
    switch (g_motorControl.mode) {
        case MOTOR_MODE_LINE_FOLLOWING:
            // 循迹模式 - 根据传感器检测到的线位置调整行驶方向
            
            // // 特殊处理：当只有最边缘的传感器检测到线时，使用更强的修正值避免振荡
            // if (g_lineTracker.sensorBits == 0b0000001) {
            //     // 只有最左边传感器检测到线，使用较强的左转修正
            //     line_correction = -10.0f;
            // } else if (g_lineTracker.sensorBits == 0b0000011) {
            //     // 左边两个传感器检测到线，使用中等左转修正
            //     line_correction = -8.0f;
            // } else if (g_lineTracker.sensorBits == 0b1000000) {
            //     // 只有最右边传感器检测到线，使用较强的右转修正
            //     line_correction = 10.0f;
            // } else if (g_lineTracker.sensorBits == 0b1100000) {
            //     // 右边两个传感器检测到线，使用中等右转修正
            //     line_correction = 8.0f;
            // } else {
            //     // 正常情况下使用PID计算
                line_correction = PID_Calculate(&g_motorControl.line_pid, g_lineTracker.linePosition);
            // }
            
            // 根据线位置偏差计算左右轮速度差值
            float correction_ratio = line_correction / LINE_PID_OUT_LIMIT;
            
            // 按比例调整左右轮速度
            left_speed_target = g_motorControl.base_speed * (1.0f - correction_ratio);
            right_speed_target = g_motorControl.base_speed * (1.0f + correction_ratio) * MOTOR_BALANCE_FACTOR;
            
            // 限制速度目标值
            left_speed_target = left_speed_target > MAX_MOTOR_SPEED ? MAX_MOTOR_SPEED : (left_speed_target < 0) ? 0 : left_speed_target;
            right_speed_target = right_speed_target > MAX_MOTOR_SPEED ? MAX_MOTOR_SPEED : (right_speed_target < 0) ? 0 : right_speed_target;
            break;
            
        case MOTOR_MODE_YAW_CORRECTION:
            // Yaw角闭环模式 - 通过调整左右轮速度差实现转向控制
            yaw_correction = PID_Calculate(&g_motorControl.yaw_pid, yaw);

            // 根据Yaw角误差计算左右轮速度差值
            left_speed_target = g_motorControl.base_speed - yaw_correction;
            right_speed_target = (g_motorControl.base_speed + yaw_correction) * MOTOR_BALANCE_FACTOR;
            
            // 限制速度目标值
            left_speed_target = (left_speed_target > MAX_MOTOR_SPEED) ? MAX_MOTOR_SPEED : (left_speed_target < -MAX_MOTOR_SPEED) ? -MAX_MOTOR_SPEED : left_speed_target;
            right_speed_target = (right_speed_target > MAX_MOTOR_SPEED) ? MAX_MOTOR_SPEED : (right_speed_target < -MAX_MOTOR_SPEED) ? -MAX_MOTOR_SPEED : right_speed_target;
            break;

        case MOTOR_MODE_SPEED_CONTROL:
            // 速度控制模式 - 直接使用设置的左右轮目标速度
            left_speed_target = g_motorControl.left_speed_target;
            right_speed_target = g_motorControl.right_speed_target;
            break;

        case MOTOR_MODE_MANUAL:
            // 手动模式下，速度由外部直接设置
            left_speed_target = g_motorControl.base_speed;
            right_speed_target = g_motorControl.base_speed * MOTOR_BALANCE_FACTOR;
            break;

        case MOTOR_MODE_STOP:
        default:
            MotorControl_Stop();
            return;
    }

    // 速度闭环控制 - 使用编码器反馈实现精确速度控制
    int32_t current_speed_L = Encoder_GetSpeed_PPS(0);
    int32_t current_speed_R = Encoder_GetSpeed_PPS(1);

    // 设置左轮速度目标并计算PID输出
    PID_SetTarget(&g_motorControl.speed_pid_L, left_speed_target);
    float pwm_L = PID_Calculate(&g_motorControl.speed_pid_L, current_speed_L);

    // 设置右轮速度目标并计算PID输出
    PID_SetTarget(&g_motorControl.speed_pid_R, right_speed_target);
    float pwm_R = PID_Calculate(&g_motorControl.speed_pid_R, current_speed_R);

    // 限制PID输出值，防止电机跑满
    pwm_L = (pwm_L > MAX_MOTOR_SPEED) ? MAX_MOTOR_SPEED : ((pwm_L < -MAX_MOTOR_SPEED) ? -MAX_MOTOR_SPEED : pwm_L);
    pwm_R = (pwm_R > MAX_MOTOR_SPEED) ? MAX_MOTOR_SPEED : ((pwm_R < -MAX_MOTOR_SPEED) ? -MAX_MOTOR_SPEED : pwm_R);

    // 设置电机PWM驱动值
    Motor_Set_Pwm(pwm_L, pwm_R);
}

/**
 * @brief 停止电机
 * 
 * 紧急停止所有电机运动，并重置所有PID控制器状态
 */
void MotorControl_Stop(void)
{
    Motor_Stop(MOTOR_ALL);
    PID_Reset(&g_motorControl.line_pid);
    PID_Reset(&g_motorControl.yaw_pid);
    PID_Reset(&g_motorControl.speed_pid_L);
    PID_Reset(&g_motorControl.speed_pid_R);
}