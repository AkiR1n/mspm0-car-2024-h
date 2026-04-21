#include "Encoder.h"

// 内部变量 - 双电机
static volatile int32_t encoder_count[2] = {0, 0};      // 编码器计数 [左电机, 右电机]
static volatile int32_t last_count[2] = {0, 0};         // 上次计数值
static volatile int32_t motor_speed_pps[2] = {0, 0};    // 当前速度（每秒脉冲数 PPS）
static volatile float motor_speed_rps[2] = {0.0f, 0.0f};    // 当前速度（每秒转速 RPS）



/**
 * @brief 初始化编码器
 */
void Encoder_Init(void)
{
    // 重置计数器
    encoder_count[0] = 0;
    encoder_count[1] = 0;
    last_count[0] = 0;
    last_count[1] = 0;
    motor_speed_pps[0] = 0;
    motor_speed_pps[1] = 0;
    motor_speed_rps[0] = 0.0f;
    motor_speed_rps[1] = 0.0f;
    
    // 清除中断状态
    DL_GPIO_clearInterruptStatus(GPIO_ENCODER_PORT, 
    GPIO_ENCODER_PIN_A1_PIN | GPIO_ENCODER_PIN_A2_PIN |
    GPIO_ENCODER_PIN_B1_PIN | GPIO_ENCODER_PIN_B2_PIN);

    // 使能NVIC中断
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
    
    // 启动定时器用于速度计算
    NVIC_EnableIRQ(TIMER_CALC_INST_INT_IRQN);
    DL_TimerA_startCounter(TIMER_CALC_INST);
}

/**
 * @brief 获取编码器计数
 * @param motor_id 电机ID (0=左电机, 1=右电机)
 * @return 编码器计数值
 */
int32_t Encoder_GetCount(uint8_t motor_id)
{
    if (motor_id >= 2) return 0;
    return encoder_count[motor_id];
}

/**
 * @brief 获取编码器速度（带方向）
 * @param motor_id 电机ID (0=左电机, 1=右电机)
 * @return 每秒脉冲数（PPS），正负表示方向
 */
int32_t Encoder_GetSpeed_PPS(uint8_t motor_id)
{
    if (motor_id >= 2) return 0;
    return motor_speed_pps[motor_id];
}

/**
 * @brief 获取编码器速度绝对值
 * @param motor_id 电机ID (0=左电机, 1=右电机)
 * @return 每秒脉冲数的绝对值
 */
uint32_t Encoder_GetSpeed_PPS_Abs(uint8_t motor_id)
{
    if (motor_id >= 2) return 0;
    return motor_speed_pps[motor_id] >= 0 ? motor_speed_pps[motor_id] : -motor_speed_pps[motor_id];
}
/**
 * @brief 获取编码器速度（带方向）
 * @param motor_id 电机ID (0=左电机, 1=右电机)
 * @return 每秒转速（RPS），正负表示方向
 */
float Encoder_GetSpeed_RPS(uint8_t motor_id)
{
    if (motor_id >= 2) return 0;
    return motor_speed_rps[motor_id];
}

/**
 * @brief 获取编码器速度绝对值
 * @param motor_id 电机ID (0=左电机, 1=右电机)
 * @return 每秒转速的绝对值
 */
float Encoder_GetSpeed_RPS_Abs(uint8_t motor_id)
{
    if (motor_id >= 2) return 0;
    return motor_speed_rps[motor_id] >= 0 ? motor_speed_rps[motor_id] : -motor_speed_rps[motor_id];
}

/**
 * @brief 重置编码器计数
 * @param motor_id 电机ID (0=左电机, 1=右电机, 2=全部)
 */
void Encoder_Reset(uint8_t motor_id)
{
    if (motor_id == 2) {
        // 重置所有电机
        encoder_count[0] = 0;
        encoder_count[1] = 0;
        last_count[0] = 0;
        last_count[1] = 0;
        motor_speed_pps[0] = 0;
        motor_speed_pps[1] = 0;
        motor_speed_rps[0] = 0.0f;
        motor_speed_rps[1] = 0.0f;
    } else if (motor_id < 2) {
        encoder_count[motor_id] = 0;
        last_count[motor_id] = 0;
        motor_speed_pps[motor_id] = 0;
        motor_speed_rps[motor_id] = 0.0f;
    }
}

/**
 * @brief 编码器GPIO中断处理函数
 */
void Encoder_IRQHandler(void)
{
    if(DL_Interrupt_getStatusGroup(DL_INTERRUPT_GROUP_1,DL_INTERRUPT_GROUP1_GPIOB)){
        // 检查是否是编码器引脚的中断
        uint32_t gpioB = DL_GPIO_getEnabledInterruptStatus(GPIO_ENCODER_PORT,
            GPIO_ENCODER_PIN_A1_PIN | GPIO_ENCODER_PIN_A2_PIN | 
            GPIO_ENCODER_PIN_B1_PIN | GPIO_ENCODER_PIN_B2_PIN);
        
        // 左电机编码器A1
        if ((gpioB & GPIO_ENCODER_PIN_A1_PIN) == GPIO_ENCODER_PIN_A1_PIN) {
            DL_GPIO_clearInterruptStatus(GPIO_ENCODER_PORT, GPIO_ENCODER_PIN_A1_PIN);
            encoder_count[0] += (Read_Encoder_A1^Read_Encoder_A2) ? 1 : -1;
        }
        // 左电机编码器A2
        if ((gpioB & GPIO_ENCODER_PIN_A2_PIN) == GPIO_ENCODER_PIN_A2_PIN) {
            DL_GPIO_clearInterruptStatus(GPIO_ENCODER_PORT, GPIO_ENCODER_PIN_A2_PIN);
            encoder_count[0] += (Read_Encoder_A1^Read_Encoder_A2) ? -1 : 1;
        }
        // 右电机编码器B1
        if ((gpioB & GPIO_ENCODER_PIN_B1_PIN) == GPIO_ENCODER_PIN_B1_PIN) {
            DL_GPIO_clearInterruptStatus(GPIO_ENCODER_PORT, GPIO_ENCODER_PIN_B1_PIN);
            encoder_count[1] += (Read_Encoder_B1^Read_Encoder_B2) ? 1 : -1;
        }
        // 右电机编码器B2
        if ((gpioB & GPIO_ENCODER_PIN_B2_PIN) == GPIO_ENCODER_PIN_B2_PIN) {
            DL_GPIO_clearInterruptStatus(GPIO_ENCODER_PORT, GPIO_ENCODER_PIN_B2_PIN);
            encoder_count[1] += (Read_Encoder_B1^Read_Encoder_B2) ? -1 : 1;
        }
    }
}

/**
 * @brief 编码器定时器中断处理函数（每10ms触发一次，用于速度计算）
 */
void Encoder_Timer_IRQHandler(void)
{
    // 计算10ms内的编码器脉冲变化量
    for (int i = 0; i < 2; i++) {
        int32_t count_diff = encoder_count[i] - last_count[i];
        
        // 定时器周期为10ms (0.01秒)，计算每秒脉冲数 (PPS)
        // 换算系数 = 1 / 0.01 = 100
        motor_speed_pps[i] = count_diff * 100;

        // 新增：计算每秒转数 (RPS)
        // RPS = PPS / 每转总脉冲数
        motor_speed_rps[i] = (float)motor_speed_pps[i] / PULSES_PER_REVOLUTION;
        
        // 保存当前计数值供下次计算使用
        last_count[i] = encoder_count[i];
    }
    
    // 注意：中断清除已移到TIMA1_IRQHandler中进行，确保中断能及时清除
    // DL_TimerA_clearInterruptStatus(TIMER_CALC_INST, DL_TIMER_INTERRUPT_ZERO_EVENT);
}
