#ifndef I2C_MPU6050_H_
#define I2C_MPU6050_H_


#include "main.h"

#define MPU_DEVICE_ADDRESS (0x68 << 1)

#define FS_GYRO_250 0
#define FS_GYRO_500 8//1 << 3
#define FS_GYRO_1000 2 << 3
#define FS_GYRO_2000 3 << 3 
#define GYRO_CALIB_SAMPLE_COUNT 1000

#define FS_ACC_2 0
#define FS_ACC_4 1 << 3
#define FS_ACC_8 2 << 3
#define FS_ACC_16 3 << 3
#define FS_ACC 4096.0f
#define ACC_CALIB_SAMPLE_COUNT 500

#define INT_RD_CLEAR 0b10000
#define DATA_RDY_EN 0b1
#define INT_OPEN 1 << 6


#define SIZE_BYTE_1 1
#define SIZE_BYTE_2 2
#define SIZE_BYTE_3 3
#define SIZE_BYTE_4 4
#define SIZE_BYTE_5 5
#define SIZE_BYTE_6 6
#define MPU_RAW_LEN 14

#define REG_CONFIG_GYRO 0x1B
#define REG_CONFIG_ACC 0x1C
#define REG_CONFIG_INT 0x37
#define REG_INT_ENABLE 0x38
#define REG_INT_STATUS 0x3A
#define REG_USR_CTRL 0x6A
#define PWR_MGMT1_REG 0x6B
#define ACCEL_REG_BASE 0x3B
#define GYRO_REG_BASE 0x43

#define REG_SMPLRT_DIV 0x19
#define REG_CONFIG     0x1A

#define DEG2RAD 0.01745329252f
#define RAD2DEG 57.295779513f
#define GYRO_SENS   65.536f 
#define ACC_SENS   4096.0f
#define GRAVITY   9.80665f

typedef struct {
    float acc_x_raw;
    float acc_y_raw;
    float acc_z_raw;
    float gyro_x_raw;
    float gyro_y_raw;
    float gyro_z_raw;
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
}Imu;

void mpu6050_init(HAL_StatusTypeDef *status);
uint8_t *mpu6050_read_acc(HAL_StatusTypeDef *status);
uint8_t *mpu6050_read_gyro(HAL_StatusTypeDef *status);
void mpu6050_test(HAL_StatusTypeDef *status);
void mpu6050_read_raw(HAL_StatusTypeDef *status, Imu* imu);
void MPU6050_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
void MPU6050_ErrorCallback(I2C_HandleTypeDef *hi2c);

#endif 