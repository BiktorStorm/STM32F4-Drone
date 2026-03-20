#ifndef CONTROL_H_
#define CONTROL_H_

#include "stm32f4xx_hal.h" 
#include "mpu6050.h"
#include "qmc5883.h"

#define PID_LOOP_HZ 500
#define DT (1.0f / (float)PID_LOOP_HZ)
#define DEADBAND_UPPER 1515
#define DEADBAND_LOWER 1485
#define MAX_THROTTLE 2000
#define LEVEL_KP  15.0f
#define MAX_ROLL_PITCH 400

void control_update(float dt, HAL_StatusTypeDef *status);


#endif