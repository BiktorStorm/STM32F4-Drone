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


void mpu6050_init(HAL_StatusTypeDef *status);
uint8_t *mpu6050_read_acc(HAL_StatusTypeDef *status);
void mpu6050_read_DMA_start(HAL_StatusTypeDef *status);
uint8_t mpu6050_read_INT_status(HAL_StatusTypeDef *status);
uint8_t *mpu6050_read_gyro(HAL_StatusTypeDef *status);
uint8_t mpu6050_ready(void);
void mpu6050_clear_ready(void);
const uint8_t* mpu6050_raw_data(void);
uint8_t mpu6050_is_busy(void);
void mpu6050_test(HAL_StatusTypeDef *status);
void motor_control_init(void);

#endif 