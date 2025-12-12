#ifndef I2C_MPU6050_H_
#define I2C_MPU6050_H_

#define MPU_DEVICE_ADDRESS 0x68 << 1

#define FS_GYRO_250 0
#define FS_GYRO_500 8//1 << 3
#define FS_GYRO_1000 2 << 3
#define FS_GYRO_2000 3 << 3 

#define REG_CONFIG_GYRO 0x1B
#define REG_CONFIG_ACC 0x1C
#define REG_USR_CTRL 0x6A

void mpu6050_init(void);


#endif 