/*
 * pid.h
 *
 *  通用PID控制器头文件
 */

#ifndef PID_H_
#define PID_H_

#include <stdint.h>

// PID控制器结构体
typedef struct {
    float Kp;                   // 比例增益
    float Ki;                   // 积分增益
    float Kd;                   // 微分增益

    float target;               // 目标值
    float actual;               // 实际值

    float error;                // 当前误差
    float last_error;           // 上一次误差
    float prev_error;           // 上上次误差，用于微分项滤波
    float integral;             // 积分累计值

    float output;               // PID输出

    float integral_limit;       // 积分限幅
    float output_limit;         // 输出限幅

} PID_Controller_t;

void PID_Init(PID_Controller_t *pid, float Kp, float Ki, float Kd, float integral_limit, float output_limit);
void PID_SetTarget(PID_Controller_t *pid, float target);
float PID_Calculate(PID_Controller_t *pid, float actual);
void PID_Reset(PID_Controller_t *pid);

#endif /* PID_H_ */