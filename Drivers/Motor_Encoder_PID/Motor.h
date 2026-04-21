#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>

/**
 * @brief 初始化电机驱动
 * @note 在调用任何其他电机函数之前，必须先调用此函数。
 */
void Motor_Init(void);

/**
 * @brief 以指定速度驱动电机
 * 
 * @param motor_id 要控制的电机ID (MOTOR_A 或 MOTOR_B)
 * @param speed    要设置的电机速度。取值范围为 -1.0 (全速反转) 到 1.0 (全速正转)。
 *                 值为 0 时将停止电机。
 */
void Motor_Run(uint8_t motor_id, float speed);

/**
 * @brief 停止指定的电机
 * 
 * @param motor_id 要停止的电机ID (MOTOR_A, MOTOR_B, 或 MOTOR_ALL)
 */
void Motor_Stop(uint8_t motor_id);

/**
 * @brief 设置双电机PWM
 * 
 * @param pwm_L 左电机PWM值 (-100.0 到 100.0)
 * @param pwm_R 右电机PWM值 (-100.0 到 100.0)
 */
void Motor_Set_Pwm(float pwm_L, float pwm_R);

// 电机ID定义
#define MOTOR_A      0      // 电机A的ID
#define MOTOR_B      1      // 电机B的ID
#define MOTOR_ALL    2      // 代表两个电机

#endif // __MOTOR_H
