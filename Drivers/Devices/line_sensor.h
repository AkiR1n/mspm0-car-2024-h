#ifndef LINE_SENSOR_H_
#define LINE_SENSOR_H_

#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint8_t bits;
    uint8_t detected;
    int16_t position;
} line_sensor_t;

void LineSensor_Init(line_sensor_t *sensor);
void LineSensor_Refresh(line_sensor_t *sensor);

#endif
