#include "servo.h"
#include "ti_msp_dl_config.h"

/* 舵机脉宽约束 */
#define SERVO_PULSE_MIN_US   500U
#define SERVO_PULSE_MAX_US   2500U
#define SERVO_PULSE_MID_US   1500U

/* 当前角度缓存，Gimbal PID 输出时需要读出当前值做增量。*/
static uint16_t s_pulse_us[SERVO_COUNT] = { SERVO_PULSE_MID_US, SERVO_PULSE_MID_US };

void Servo_Init(void)
{
#if defined PWM_SERVO_INST
    /* 假设 SysConfig 配置为 1 MHz counter + 20000 load → 20 ms period（50 Hz）。
     * 此时 CC 值的单位就是 µs，直接写脉宽即可。*/
    Servo_SetPulseUs(SERVO_PAN,  SERVO_PULSE_MID_US);
    Servo_SetPulseUs(SERVO_TILT, SERVO_PULSE_MID_US);
    DL_TimerG_startCounter(PWM_SERVO_INST);
#endif
}

void Servo_SetAngle(Servo_Channel_t ch, float deg)
{
    if (deg < 0.0f)   deg = 0.0f;
    if (deg > 180.0f) deg = 180.0f;
    uint16_t us = (uint16_t)(SERVO_PULSE_MIN_US +
                             (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US) * deg / 180.0f);
    Servo_SetPulseUs(ch, us);
}

void Servo_SetPulseUs(Servo_Channel_t ch, uint16_t us)
{
    if (ch >= SERVO_COUNT) return;
    if (us < SERVO_PULSE_MIN_US) us = SERVO_PULSE_MIN_US;
    if (us > SERVO_PULSE_MAX_US) us = SERVO_PULSE_MAX_US;
    s_pulse_us[ch] = us;

#if defined PWM_SERVO_INST
    uint32_t cc_idx = (ch == SERVO_PAN) ? DL_TIMER_CC_0_INDEX : DL_TIMER_CC_1_INDEX;
    /* 向上计数模式下，CC 值 = load - pulse。此处假设 load 已被 SysConfig 设为 20000。*/
    uint32_t load = DL_TimerG_getLoadValue(PWM_SERVO_INST);
    uint32_t cc   = (load > us) ? (load - us) : 0;
    DL_TimerG_setCaptureCompareValue(PWM_SERVO_INST, cc, cc_idx);
#endif
}
