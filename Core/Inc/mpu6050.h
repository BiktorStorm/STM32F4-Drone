#ifndef I2C_MPU6050_H_
#define I2C_MPU6050_H_


#include "main.h"

#define MPU_DEVICE_ADDRESS (0x68 << 1)

#define FS_GYRO_250 0
#define FS_GYRO_500 8//1 << 3
#define FS_GYRO_1000 2 << 3
#define FS_GYRO_2000 3 << 3 

#define FS_ACC_2 0
#define FS_ACC_4 1 << 3
#define FS_ACC_8 2 << 3
#define FS_ACC_16 3 << 3

#define SIZE_BYTE_1 1
#define SIZE_BYTE_2 2
#define SIZE_BYTE_3 3
#define SIZE_BYTE_4 4
#define SIZE_BYTE_5 5
#define SIZE_BYTE_6 6

#define REG_CONFIG_GYRO 0x1B
#define REG_CONFIG_ACC 0x1C
#define REG_USR_CTRL 0x6A
#define PWR_MGMT1_REG 0x6B
#define ACCEL_REG_BASE 0x3B
#define GYRO_REG_BASE 0x43

void mpu6050_init(void);
uint8_t *mpu6050_read_acc(HAL_StatusTypeDef *status);
uint8_t *mpu6050_read_gyro(HAL_StatusTypeDef *status);


#endif 