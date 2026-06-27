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


//GPT RTH defines
#define RTH_HOME_REACHED_M         2.5f
#define RTH_START_BRAKE_M          8.0f

#define RTH_YAW_KP                 2.0f
#define RTH_MAX_YAW_RATE           90.0f      // deg/s command into yaw PID

#define RTH_FORWARD_CMD_US         120.0f     // strong forward pitch command
#define RTH_FORWARD_SLOW_CMD_US    70.0f      // slower when close to home
#define RTH_YAW_ALIGN_DEG          20.0f      // only move forward if roughly pointed home
#define RTH_CLIMB_ALTITUDE_M       120.0f
#define RTH_ALTITUDE_TOLERANCE_M   3.0f
#define RTH_HOME_ELEVATION_TOL_M   2.0f
#define RTH_CLIMB_THROTTLE_BOOST   140.0f
#define RTH_DESCEND_THROTTLE_REDUCE 80.0f
#define RTH_ALT_HOLD_KP_US_PER_M   4.0f
#define RTH_ALT_HOLD_MAX_CORR_US   120.0f

typedef enum {
    PERIPH_READY = 1,
    PERIPH_NOT_READY = 0
} Periph_status;

void control_update(float dt, HAL_StatusTypeDef *status, Periph_status periph_status);


#endif
