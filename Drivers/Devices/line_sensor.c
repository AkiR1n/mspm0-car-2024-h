#include "line_sensor.h"

#include "linetracker.h"

void LineSensor_Init(line_sensor_t *sensor)
{
    if (sensor == NULL) {
        return;
    }

    LineTracker_Init();
    sensor->bits = 0u;
    sensor->detected = 0u;
    sensor->position = 0;
}

void LineSensor_Refresh(line_sensor_t *sensor)
{
    if (sensor == NULL) {
        return;
    }

    LineTracker_ReadSensors();
    sensor->bits = LineTracker_GetSensorBits();
    sensor->detected = (uint8_t)LineTracker_IsLineDetected();
    sensor->position = LineTracker_GetLinePosition();
}
