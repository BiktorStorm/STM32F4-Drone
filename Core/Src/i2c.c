#include "i2c.h"
#include "main.h"
#include "mpu6050.h"
#include "bmp280.h"



static volatile I2C_Owner_t hi2c1_owner = I2C_OWNER_NONE;

void I2C_Dispatch_SetOwner(I2C_HandleTypeDef *hi2c, I2C_Owner_t owner)
{
    if (hi2c->Instance == hi2c1.Instance)
    {
        hi2c1_owner = owner;
    }
}

I2C_Owner_t I2C_Dispatch_GetOwner(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == hi2c1.Instance)
    {
        return hi2c1_owner;
    }

    return I2C_OWNER_NONE;
}

/* HAL callbacks */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != hi2c1.Instance) return;

    switch (hi2c1_owner)
    {
        case I2C_OWNER_MPU6050:
            MPU6050_MemRxCpltCallback(hi2c);
            break;

        case I2C_OWNER_BMP280:
            BMP280_MemRxCpltCallback(hi2c);
            break;

        default:
            break;
    }

    hi2c1_owner = I2C_OWNER_NONE;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != hi2c1.Instance) return;

    switch (hi2c1_owner)
    {
        case I2C_OWNER_MPU6050:
            MPU6050_ErrorCallback(hi2c);
            break;

        case I2C_OWNER_BMP280:
            BMP280_ErrorCallback(hi2c);
            break;

        default:
            break;
    }

    hi2c1_owner = I2C_OWNER_NONE;
}