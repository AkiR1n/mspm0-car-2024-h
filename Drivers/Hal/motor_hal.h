#ifndef MOTOR_HAL_H_
#define MOTOR_HAL_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    MOTOR_HAL_LEFT = 0,
    MOTOR_HAL_RIGHT = 1,
} motor_hal_id_t;

typedef enum {
    MOTOR_HAL_DIR_COAST = 0,
    MOTOR_HAL_DIR_FORWARD = 1,
    MOTOR_HAL_DIR_REVERSE = -1,
} motor_hal_dir_t;

void MotorHal_Init(void);
void MotorHal_SetDirection(motor_hal_id_t id, motor_hal_dir_t dir);
void MotorHal_SetDuty(motor_hal_id_t id, float duty);

#endif
