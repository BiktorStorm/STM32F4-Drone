#include "mpu6050.h"
#include <main.h>
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_i2c.h"
#include "usbd_cdc_if.h"    //Only here so that false error disappears with CDC_transmit_FS function

static uint8_t mpu_raw[14];

static volatile uint8_t mpu_read_ready = 0;
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
    *status = HAL_ERROR;
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
  *status = HAL_ERROR;
  return;
  }
  
  // temp_data = INT_RD_CLEAR;
  // ret = HAL_I2C_Mem_Write(&hi2c1, MPU_DEVICE_ADDRESS, REG_CONFIG_INT, I2C_MEMADD_SIZE_8BIT, &temp_data, sizeof(temp_data), 100);
  // if(ret == HAL_OK){
  // uint8_t buffer[] = "CONFIG_INT configured\n";
  // CDC_Transmit_FS(buffer, sizeof(buffer));
  // } else {
  // uint8_t buffer1[] = "Cannot configure CONFIG_INT, returning form initialization\n";
  // CDC_Transmit_FS(buffer1, sizeof(buffer1));
  // return;
  // }

  // temp_data = DATA_RDY_EN;
  // ret = HAL_I2C_Mem_Write(&hi2c1, MPU_DEVICE_ADDRESS, REG_INT_ENABLE, I2C_MEMADD_SIZE_8BIT, &temp_data, sizeof(temp_data), 100);
  // if(ret == HAL_OK){
  // uint8_t buffer[] = "INT_ENABLE configured\n";
  // CDC_Transmit_FS(buffer, sizeof(buffer));
  // } else {
  // uint8_t buffer1[] = "Cannot configure INT_ENABLE, returning form initialization\n";
  // CDC_Transmit_FS(buffer1, sizeof(buffer1));
  // return;
  // }

  *status = HAL_OK;
}

uint8_t *mpu6050_read_acc(HAL_StatusTypeDef *status) {
  static uint8_t data[6]; 
  *status = HAL_I2C_Mem_Read(&hi2c1, MPU_DEVICE_ADDRESS , ACCEL_REG_BASE, I2C_MEMADD_SIZE_8BIT, data, SIZE_BYTE_6, 100);
  return data;
}

void mpu6050_read_DMA_start(HAL_StatusTypeDef *status) {
  if(mpu_busy) {
    *status = HAL_BUSY;
    return;
  }
  mpu_busy = 1;
  mpu_read_ready = 0;
  
  *status = HAL_I2C_Mem_Read_DMA(&hi2c1, MPU_DEVICE_ADDRESS , ACCEL_REG_BASE, I2C_MEMADD_SIZE_8BIT, mpu_raw, MPU_RAW_LEN);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != hi2c1.Instance) return;
    mpu_busy = 0;
    mpu_read_ready = 1;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != hi2c1.Instance) return;
    mpu_busy = 0;
    mpu_read_ready = 0;
}

uint8_t *mpu6050_read_gyro(HAL_StatusTypeDef *status) {
  static uint8_t data[6]; 
  *status = HAL_I2C_Mem_Read(&hi2c1, MPU_DEVICE_ADDRESS ,GYRO_REG_BASE, I2C_MEMADD_SIZE_8BIT, data, SIZE_BYTE_6, 100);
  return data;
}

uint8_t mpu6050_read_INT_status(HAL_StatusTypeDef *status) {
  uint8_t data;
  *status = HAL_I2C_Mem_Read(&hi2c1, MPU_DEVICE_ADDRESS , REG_INT_STATUS, I2C_MEMADD_SIZE_8BIT, &data, sizeof(data), 10); 
  return data;
}

void mpu6050_test(HAL_StatusTypeDef *status){
  int16_t acc_X = 0;
  int16_t acc_Y = 0;
  int16_t acc_Z = 0;
  int16_t gyro_X = 0;
  int16_t gyro_Y = 0;
  int16_t gyro_Z = 0;
  
  if(!mpu6050_is_busy() && !mpu6050_ready()) {
    mpu6050_read_DMA_start(status);
  }
  
  if(mpu6050_ready()) {
    mpu6050_clear_ready();
    const uint8_t *buffer = mpu6050_raw_data(); //temperature available at indeces: 6 and 7
    acc_X = (int16_t)((buffer[0] << 8) | (buffer[1]));
    acc_Y = (int16_t)((buffer[2] << 8) | (buffer[3]));
    acc_Z = (int16_t)((buffer[4] << 8) | (buffer[5]));

    gyro_X = (int16_t)((buffer[8] << 8) | (buffer[9]));
    gyro_Y = (int16_t)((buffer[10] << 8) | (buffer[11]));
    gyro_Z = (int16_t)((buffer[12] << 8) | (buffer[13]));
    
    if(*status == HAL_OK) {
      char cdc_buf[64];
      int len = snprintf(cdc_buf, sizeof(cdc_buf), "ACC: X =%d Y=%d Z=%d\r\n Gyro: X =%d Y=%d Z=%d\r\n", acc_X, acc_Y, acc_Z, gyro_X, gyro_Y, gyro_Z);
      if(len > 0){
        if (len > sizeof(cdc_buf)) {
          len = sizeof(cdc_buf);  
        }
        // CDC_Transmit_FS((uint8_t*)cdc_buf, len);
        while (CDC_Transmit_FS((uint8_t*)cdc_buf, len) == USBD_BUSY) {
          HAL_Delay(1);
        }
      }
    }
  } 
}

uint8_t mpu6050_ready(void) { return mpu_read_ready; }
void mpu6050_clear_ready(void) { mpu_read_ready = 0; }
const uint8_t* mpu6050_raw_data(void) { return mpu_raw; }

uint8_t mpu6050_is_busy(void) { return mpu_busy; }