#ifndef CHASSIS_SYSTEM_H_
#define CHASSIS_SYSTEM_H_

#include "chassis.h"
#include "encoder_drv.h"
#include "imu_drv.h"
#include "line_controller.h"
#include "line_sensor.h"
#include "motor_drv.h"
#include "wheel.h"
#include "yaw_controller.h"

void chassis_system_init(void);
chassis_t *chassis_system_get_chassis(void);
motor_t *chassis_system_get_left_motor(void);
motor_t *chassis_system_get_right_motor(void);
encoder_t *chassis_system_get_left_encoder(void);
encoder_t *chassis_system_get_right_encoder(void);
imu_t *chassis_system_get_imu(void);
line_sensor_t *chassis_system_get_line_sensor(void);
yaw_controller_t *chassis_system_get_yaw_controller(void);
line_controller_t *chassis_system_get_line_controller(void);

#endif
