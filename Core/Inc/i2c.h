#ifndef I2C_H_
#define I2C_H_

#include "stm32f4xx_hal.h"

#include "main.h"

extern I2C_HandleTypeDef hi2c1;

typedef enum
{
    I2C_OWNER_NONE = 0,
    I2C_OWNER_MPU6050,
    I2C_OWNER_BMP280,
    I2C_OWNER_QMC5883
} I2C_Owner_t;

void I2C_Dispatch_SetOwner(I2C_HandleTypeDef *hi2c, I2C_Owner_t owner);
I2C_Owner_t I2C_Dispatch_GetOwner(I2C_HandleTypeDef *hi2c);

#endif