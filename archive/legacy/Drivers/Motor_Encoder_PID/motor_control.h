/*
 * motor_control.h
 *
 *  电机控制头文件 - 多模式电机控制系统
 *  
 *  设计理念：
 *  - 统一的PID控制框架
 *  - 多种控制模式适应不同场景
 *  - 针对循迹小车优化的参数
 */

#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

#include "pid.h"
#include "linetracker.h"
#include "mpu6050.h"
#include "Motor.h"

// 电机控制模式
typedef enum {
    MOTOR_MODE_STOP,            // 停止
    MOTOR_MODE_LINE_FOLLOWING,  // 循迹模式 - 基于7路传感器的PID控制
    MOTOR_MODE_YAW_CORRECTION,  // Yaw角闭环 - 基于MPU6050的方向保持
    MOTOR_MODE_SPEED_CONTROL,   // 速度控制模式 - 直接设置左右轮速度
    MOTOR_MODE_MANUAL           // 手动控制
} Motor_Mode_t;

// 电机控制结构体
typedef struct {
    PID_Controller_t line_pid;      // 循迹PID控制器
    PID_Controller_t yaw_pid;       // Yaw角PID控制器
    PID_Controller_t speed_pid_L;   // 左轮速度PID控制器
    PID_Controller_t speed_pid_R;   // 右轮速度PID控制器

    Motor_Mode_t mode;              // 当前控制模式

    float base_speed;               // 基础速度
    float target_yaw;               // 目标Yaw角
    float left_speed_target;        // 左轮目标速度
    float right_speed_target;       // 右轮目标速度

} Motor_Control_t;

// 全局变量声明
extern Motor_Control_t g_motorControl;

// 核心控制函数
void MotorControl_Init(void);
void MotorControl_SetMode(Motor_Mode_t mode);
void MotorControl_SetBaseSpeed(float speed);
void MotorControl_Update(void);
void MotorControl_Stop(void);

// 专用控制函数
void MotorControl_SetTargetYaw(float yaw);                              // Yaw角控制
void MotorControl_SetSpeedTarget(float left_speed, float right_speed);  // 直接速度控制

#endif /* MOTOR_CONTROL_H_ */