#include "Motor.h"
#include "ti_msp_dl_config.h"

// =====================================================================================================================
// 宏定义
// =====================================================================================================================

#define MIN_PWM_DUTY 0.1f   // 最小有效PWM占空比，防止电机在极低速度下不转或抖动
#define MOTOR_A      0      // 电机A的ID
#define MOTOR_B      1      // 电机B的ID
#define MOTOR_ALL    2      // 代表两个电机

// =====================================================================================================================
// 内部变量
// =====================================================================================================================

static uint8_t pwm_running = 0; // PWM定时器运行状态标志 (1: 运行, 0: 停止)

// =====================================================================================================================
// 内部(私有)函数声明
// =====================================================================================================================

static void Motor_PWM_Start(void);
static void Motor_SetDirection(uint8_t motor_id, int8_t direction);
static void Motor_SetPWM(uint8_t motor_id, float duty);

// =====================================================================================================================
// 公共函数定义
// =====================================================================================================================

/**
 * @brief 初始化电机驱动模块
 * @note  在调用任何其他电机函数之前，必须先调用此函数。
 *        此函数会停止所有电机并启动PWM定时器。
 */
void Motor_Init(void)
{
    Motor_Stop(MOTOR_ALL);
    Motor_PWM_Start();
}

/**
 * @brief 以指定速度驱动电机
 * 
 * @param motor_id 要控制的电机ID (MOTOR_A 或 MOTOR_B)
 * @param speed    要设置的电机速度。取值范围为 -1.0 (全速反转) 到 1.0 (全速正转)。
 *                 值为 0 时将调用 Motor_Stop() 来停止电机。
 */
void Motor_Run(uint8_t motor_id, float speed)
{
    if (speed == 0.0f) {
        Motor_Stop(motor_id);
        return;
    }

    // 限制速度输入范围
    if (speed > 1.0f)  speed = 1.0f;
    if (speed < -1.0f) speed = -1.0f;

    // 根据速度符号确定方向和绝对速度值
    float abs_speed = (speed > 0) ? speed : -speed;
    int8_t direction = (speed > 0) ? 1 : -1;

    // 如果速度不为0，但小于最小有效占空比，则强制设置为最小有效占空比
    if (abs_speed > 0.0f && abs_speed < MIN_PWM_DUTY) {
        abs_speed = MIN_PWM_DUTY;
    }

    Motor_SetDirection(motor_id, direction);
    Motor_SetPWM(motor_id, abs_speed);
}

/**
 * @brief 停止指定的电机
 * 
 * @param motor_id 要停止的电机ID (MOTOR_A, MOTOR_B, 或 MOTOR_ALL)
 */
void Motor_Stop(uint8_t motor_id)
{
    if (motor_id == MOTOR_ALL) {
        // 停止两个电机
        Motor_SetDirection(MOTOR_A, 0);
        Motor_SetPWM(MOTOR_A, 0.0f);
        Motor_SetDirection(MOTOR_B, 0);
        Motor_SetPWM(MOTOR_B, 0.0f);
    } else {
        // 停止单个电机
        Motor_SetDirection(motor_id, 0);
        Motor_SetPWM(motor_id, 0.0f);
    }
}

// =====================================================================================================================
// 内部(私有)函数定义
// =====================================================================================================================

/**
 * @brief 启动PWM定时器
 * @note  内部函数，防止重复启动。
 */
static void Motor_PWM_Start(void)
{
    if (!pwm_running) {
        DL_TimerA_startCounter(PWM_MOTOR_INST);
        pwm_running = 1;
    }
}

/**
 * @brief 设置单个电机的方向
 * @note  内部函数，通过控制GPIO实现。
 * @param motor_id  电机ID (MOTOR_A 或 MOTOR_B)
 * @param direction 方向 (1: 正转, -1: 反转, 0: 停止/高阻态)
 */
static void Motor_SetDirection(uint8_t motor_id, int8_t direction)
{
    if (motor_id == MOTOR_A) {
        switch (direction) {
            case 1:  // 正转
                DL_GPIO_setPins(GPIO_MOTOR_PORT, GPIO_MOTOR_PIN_AIN1_PIN);
                DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_PIN_AIN2_PIN);
                break;
            case -1: // 反转
                DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_PIN_AIN1_PIN);
                DL_GPIO_setPins(GPIO_MOTOR_PORT, GPIO_MOTOR_PIN_AIN2_PIN);
                break;
            default: // 停止
                DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_PIN_AIN1_PIN | GPIO_MOTOR_PIN_AIN2_PIN);
                break;
        }
    } else if (motor_id == MOTOR_B) {
        switch (direction) {
            case 1:  // 正转
                DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_PIN_BIN1_PIN);
                DL_GPIO_setPins(GPIO_MOTOR_PORT, GPIO_MOTOR_PIN_BIN2_PIN);
                break;
            case -1: // 反转
                DL_GPIO_setPins(GPIO_MOTOR_PORT, GPIO_MOTOR_PIN_BIN1_PIN);
                DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_PIN_BIN2_PIN);
                break;
            default: // 停止
                DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_PIN_BIN1_PIN | GPIO_MOTOR_PIN_BIN2_PIN);
                break;
        }
    }
}

/**
 * @brief 设置单个电机的PWM占空比
 * @note  内部函数。
 * @param motor_id 电机ID (MOTOR_A 或 MOTOR_B)
 * @param duty     占空比 (0.0 到 1.0)
 */
static void Motor_SetPWM(uint8_t motor_id, float duty)
{
    // 约束占空比在有效范围
    if (duty > 1.0f) duty = 1.0f;
    if (duty < 0.0f) duty = 0.0f;

    // 动态获取定时器周期值，以防被其他地方修改
    uint32_t period = DL_TimerA_getLoadValue(PWM_MOTOR_INST);

    // 计算PWM比较寄存器的值。占空比越大，比较值越小（向上计数模式）
    uint32_t pwm_value = (uint32_t)(period - (period * duty));
    
    // 根据电机ID设置对应PWM通道的比较值
    if (motor_id == MOTOR_A) {
        DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, pwm_value, DL_TIMER_CC_0_INDEX);
    } else if (motor_id == MOTOR_B) {
        DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, pwm_value, DL_TIMER_CC_1_INDEX);
    }
}

/**
 * @brief 设置双电机PWM
 * 
 * @param pwm_L 左电机PWM值 (-100.0 到 100.0)
 * @param pwm_R 右电机PWM值 (-100.0 到 100.0)
 */
void Motor_Set_Pwm(float pwm_L, float pwm_R)
{
    // 将PWM值从-100~100范围转换为-1.0~1.0范围
    float speed_L = pwm_L / 100.0f;
    float speed_R = pwm_R / 100.0f;
    
    // 调用现有的Motor_Run函数
    Motor_Run(MOTOR_A, speed_L);
    Motor_Run(MOTOR_B, speed_R);
}
