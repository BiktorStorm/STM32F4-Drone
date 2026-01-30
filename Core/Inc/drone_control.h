#ifndef DRONE_CONTROL_H_
#define DRONE_CONTROL_H_
#include "main.h"

#define GYRO_LSB_PER_DPS 65.536f     //need to doublecheck value I have 500 DPS
#define ACC_LSB_PER_G 8192.0f
#define ESC_MIN_US 1000
#define ESC_MAX_US 2000
#define ESC_IDLE_US 1050
#define ESC_TEST_MAX_US 1400   // SAFETY CAP for testing

void control_update(float dt, HAL_StatusTypeDef* status);

#endif