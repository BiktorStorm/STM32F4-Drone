#include "mpu6050.h"
#include <main.h>
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_i2c.h"
#include "usbd_cdc_if.h"    //Only here so that false error disappears with CDC_transmit_FS function

static uint8_t mpu_acc_raw[6];
static volatile uint8_t mpu_acc_ready = 0;
static volatile uint8_t mpu_busy = 0;

extern I2C_HandleTypeDef hi2c1;

void mpu6050_init(HAL_StatusTypeDef *status) {
  HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&hi2c1, MPU_DEVICE_ADDRESS, 1, 100);
  if(ret == HAL_OK){
    uint8_t buffer[] = "MPU-6050 connected\n";
    CDC_Transmit_FS(buffer, sizeof(buffer));
  } else {
    uint8_t buffer1[] = "Cannot connect to MPU-6050, returning from initialization\n";
    CDC_Transmit_FS(buffer1, sizeof(buffer1));
    return;
  }

  uint8_t temp_data = FS_GYRO_500;
  ret = HAL_I2C_Mem_Write(&hi2c1, MPU_DEVICE_ADDRESS, REG_CONFIG_GYRO, I2C_MEMADD_SIZE_8BIT, &temp_data, sizeof(temp_data), 100);
  if(ret == HAL_OK){
    uint8_t buffer[] = "Gyro configured\n";
    CDC_Transmit_FS(buffer, sizeof(buffer));
  } else {
    uint8_t buffer1[] = "Cannot configure gyro, returning form initialization\n";
    CDC_Transmit_FS(buffer1, sizeof(buffer1));
    *status = HAL_ERROR;
    return;
  }

  temp_data = FS_ACC_4;
  ret = HAL_I2C_Mem_Write(&hi2c1, MPU_DEVICE_ADDRESS, REG_CONFIG_ACC, I2C_MEMADD_SIZE_8BIT, &temp_data, sizeof(temp_data), 100);
  if(ret == HAL_OK){
  uint8_t buffer[] = "Accelerometer configured\n";
  CDC_Transmit_FS(buffer, sizeof(buffer));
  } else {
  uint8_t buffer1[] = "Cannot configure ACCELEROMETER, returning form initialization\n";
  CDC_Transmit_FS(buffer1, sizeof(buffer1));
  *status = HAL_ERROR;
  return;
  }

  temp_data = 0;
  ret = HAL_I2C_Mem_Write(&hi2c1, MPU_DEVICE_ADDRESS, PWR_MGMT1_REG, I2C_MEMADD_SIZE_8BIT, &temp_data, sizeof(temp_data), 100);
  if(ret == HAL_OK){
  uint8_t buffer[] = "PWR_MGMT1 configured\n";
  CDC_Transmit_FS(buffer, sizeof(buffer));
  } else {
  uint8_t buffer1[] = "Cannot configure PWR_MGMT1, returning form initialization\n";
  CDC_Transmit_FS(buffer1, sizeof(buffer1));
  }
  *status = HAL_OK;
}

uint8_t *mpu6050_read_acc(HAL_StatusTypeDef *status) {
  static uint8_t data[6]; 
  *status = HAL_I2C_Mem_Read(&hi2c1, MPU_DEVICE_ADDRESS + 1, ACCEL_REG_BASE, I2C_MEMADD_SIZE_8BIT, data, SIZE_BYTE_6, 100);
  return data;
}

void mpu6050_read_acc_DMA_start(HAL_StatusTypeDef *status) {
  if(mpu_busy) {
    *status = HAL_BUSY;
    return;
  }

  mpu_busy = 1;
  mpu_acc_ready = 0;
  *status = HAL_I2C_Mem_Read_DMA(&hi2c1, MPU_DEVICE_ADDRESS + 1, ACCEL_REG_BASE, I2C_MEMADD_SIZE_8BIT, mpu_acc_raw, SIZE_BYTE_6);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != hi2c1.Instance) return;

    mpu_busy = 0;
    mpu_acc_ready = 1;
}

uint8_t *mpu6050_read_gyro(HAL_StatusTypeDef *status) {
  static uint8_t data[6]; 
  *status = HAL_I2C_Mem_Read(&hi2c1, MPU_DEVICE_ADDRESS + 1,GYRO_REG_BASE, I2C_MEMADD_SIZE_8BIT, data, SIZE_BYTE_6, 100);
  return data;
}

