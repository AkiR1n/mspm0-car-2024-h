#ifndef MOTOR_DRV_H_
#define MOTOR_DRV_H_

#include <stddef.h>
#include <stdint.h>

#include "motor_hal.h"

typedef struct {
    motor_hal_id_t hal_id;
    float          min_duty;
} motor_cfg_t;

typedef struct {
    motor_cfg_t cfg;
    float       applied_duty;
} motor_t;

void Motor_Init(motor_t *motor, const motor_cfg_t *cfg);
void Motor_SetDuty(motor_t *motor, float duty);
void Motor_Stop(motor_t *motor);
float Motor_GetAppliedDuty(const motor_t *motor);

#endif
