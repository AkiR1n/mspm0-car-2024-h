/*
 * Servo.h — 50 Hz PWM 舵机驱动（MG996R 系列）
 *
 * 脉宽 500–2500 µs ↔ 0°–180°。
 * 具体 PWM 硬件（定时器、CC 通道、引脚）由 SysConfig 决定，本模块只负责
 * 把角度转成 CC 寄存器值并写入。
 *
 * 调用链设想：
 *   Servo_Init()                       // 启动 PWM 定时器
 *   Servo_SetAngle(SERVO_PAN, 90.0f)   // 水平舵机 90°
 *   Servo_SetAngle(SERVO_TILT, 60.0f)  // 垂直舵机 60°
 *
 * SysConfig 要求（用户在 CCS Theia GUI 里添加）：
 *   - 新建一个 TIMG 实例，$name = "PWM_SERVO"
 *   - timerCount ≈ 40000（80 MHz PCLK, 1MHz counter, 20 ms period → PRE=80）
 *   - 2 个 PWM 通道：CCP0=SERVO_PAN, CCP1=SERVO_TILT
 *   - 引脚绑定到 LaunchPad 外沿，避免和 PWM_MOTOR / I²C 冲突
 *
 * 在 ti_msp_dl_config.h 中应生成：
 *   PWM_SERVO_INST                  (e.g. TIMG0)
 *   GPIO_PWM_SERVO_C0_IDX           (通道 0)
 *   GPIO_PWM_SERVO_C1_IDX           (通道 1)
 */
#ifndef _SERVO_H_
#define _SERVO_H_

#include <stdint.h>

typedef enum {
    SERVO_PAN  = 0,   // 水平舵机
    SERVO_TILT = 1,   // 垂直舵机
    SERVO_COUNT
} Servo_Channel_t;

void Servo_Init(void);
void Servo_SetAngle(Servo_Channel_t ch, float deg);     // 0–180
void Servo_SetPulseUs(Servo_Channel_t ch, uint16_t us); // 500–2500

#endif  /* _SERVO_H_ */
