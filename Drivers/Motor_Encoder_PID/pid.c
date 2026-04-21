/*
 * pid.c
 *
 *  通用PID控制器实现
 */

#include "pid.h"

/**
 * @brief 初始化PID控制器
 * @param pid 指向PID控制器结构体的指针
 * @param Kp 比例增益
 * @param Ki 积分增益
 * @param Kd 微分增益
 * @param integral_limit 积分限幅
 * @param output_limit 输出限幅
 */
void PID_Init(PID_Controller_t *pid, float Kp, float Ki, float Kd, float integral_limit, float output_limit)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
    pid->target = 0.0f;
    pid->actual = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->prev_error = 0.0f;  // 用于微分项滤波
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

/**
 * @brief 设置PID目标值
 * @param pid 指向PID控制器结构体的指针
 * @param target 目标值
 */
void PID_SetTarget(PID_Controller_t *pid, float target)
{
    pid->target = target;
}

/**
 * @brief 计算PID输出
 * @param pid 指向PID控制器结构体的指针
 * @param actual 实际值
 * @return PID输出值
 */
float PID_Calculate(PID_Controller_t *pid, float actual)
{
    pid->actual = actual;
    pid->error = pid->target - pid->actual;

    // 积分项计算和抗积分饱和处理
    // 只有在输出未饱和时才累加积分项，或者误差与输出同方向时才累加
    float integral_temp = pid->integral + pid->error;
    
    // 积分限幅
    if (integral_temp > pid->integral_limit) {
        integral_temp = pid->integral_limit;
    } else if (integral_temp < -pid->integral_limit) {
        integral_temp = -pid->integral_limit;
    }
    
    // 抗积分饱和：只有当控制器输出未达到极限值，或者误差信号与输出同方向时才更新积分项
    float output_p = pid->Kp * pid->error;
    float output_i = pid->Ki * integral_temp;
    float output_d = pid->Kd * (pid->error - pid->last_error);
    
    float output_total = output_p + output_i + output_d;
    
    // 只有在输出未饱和时才更新积分项，防止积分饱和
    if ((output_total >= -pid->output_limit && output_total <= pid->output_limit) ||
        (pid->error * output_i > 0)) {
        pid->integral = integral_temp;
    }

    // PID计算
    pid->output = output_p + pid->Ki * pid->integral + output_d;

    // 输出限幅
    if (pid->output > pid->output_limit) {
        pid->output = pid->output_limit;
    } else if (pid->output < -pid->output_limit) {
        pid->output = -pid->output_limit;
    }

    pid->last_error = pid->error;

    return pid->output;
}

/**
 * @brief 重置PID控制器状态
 * @param pid 指向PID控制器结构体的指针
 */
void PID_Reset(PID_Controller_t *pid)
{
    pid->target = 0.0f;
    pid->actual = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}