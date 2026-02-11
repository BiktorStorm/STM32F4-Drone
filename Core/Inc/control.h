#ifndef CONTROL_H_
#define CONTROL_H_

#include "stm32f4xx_hal.h" 

#define PID_LOOP_HZ 500
#define DT (1.0f / (float)PID_LOOP_HZ)
#define DEADBAND_UPPER 1510
#define DEADBAND_LOWER 1490
#define MAX_THROTTLE 1400
#define LEVEL_KP  15.0f
#define MAX_ROLL_PITCH 400

void control_update(float dt, HAL_StatusTypeDef *status);

#endif