#include "mpu6050.h"
#include <main.h>
#include <stdint.h>
#include "i2c.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_i2c.h"
#include "usbd_cdc_if.h"    //Only here so that false error disappears with CDC_transmit_FS function
#include <math.h>

static uint8_t mpu_raw[14];

static volatile uint8_t mpu_read_ready = 0;
static volatile uint8_t mpu_busy = 0;
volatile int16_t gyro_error_X = 0;
volatile int16_t gyro_error_Y = 0;
volatile int16_t gyro_error_Z = 0;
volatile int16_t acc_error_X = 0;
volatile int16_t acc_error_Y = 0;
extern I2C_HandleTypeDef hi2c1;

uint8_t mpu6050_is_busy(void) { return mpu_busy; }
uint8_t mpu6050_ready(void) { return mpu_read_ready; }
void mpu6050_clear_ready(void) { mpu_read_ready = 0; }
const uint8_t* mpu6050_raw_data(void) { return mpu_raw; }

void mpu6050_read_DMA_start(HAL_StatusTypeDef *status) {
  if(mpu_busy || I2C_Dispatch_GetOwner(&hi2c1) != I2C_OWNER_NONE) {
    *status = HAL_BUSY;
    return;
  }
  mpu_busy = 1;
  mpu_read_ready = 0;

  I2C_Dispatch_SetOwner(&hi2c1, I2C_OWNER_MPU6050);
  
  *status = HAL_I2C_Mem_Read_DMA(&hi2c1, MPU_DEVICE_ADDRESS , ACCEL_REG_BASE, I2C_MEMADD_SIZE_8BIT, mpu_raw, MPU_RAW_LEN);
}

void gyro_calibrate(HAL_StatusTypeDef *status) {
  uint16_t i = 0;
  int32_t sum_X = 0;
  int32_t sum_Y = 0;
  int32_t sum_Z = 0;
  while(i < GYRO_CALIB_SAMPLE_COUNT) {
    if(!mpu6050_is_busy() && !mpu6050_ready()) {
      mpu6050_read_DMA_start(status);
    }
    if(mpu6050_ready()) {
      mpu6050_clear_ready();
      const uint8_t *buffer = mpu6050_raw_data(); //temperature available at indeces: 6 and 7
      int16_t gyro_X = (int16_t)((buffer[8] << 8) | (buffer[9]));
      int16_t gyro_Y = (int16_t)((buffer[10] << 8) | (buffer[11]));
      int16_t gyro_Z = (int16_t)((buffer[12] << 8) | (buffer[13]));

      sum_X = sum_X + gyro_X;
      sum_Y = sum_Y + gyro_Y;
      sum_Z = sum_Z + gyro_Z;
      i++;
    }
  }
  gyro_error_X = sum_X / GYRO_CALIB_SAMPLE_COUNT;
  gyro_error_Y = sum_Y / GYRO_CALIB_SAMPLE_COUNT;
  gyro_error_Z = sum_Z / GYRO_CALIB_SAMPLE_COUNT;
}

void acc_calibrate(HAL_StatusTypeDef *status) {
  uint16_t i = 0;
  int32_t sum_X = 0;
  int32_t sum_Y = 0;
  int32_t sum_Z = 0;
  while(i < ACC_CALIB_SAMPLE_COUNT) {
    if(!mpu6050_is_busy() && !mpu6050_ready()) {
      mpu6050_read_DMA_start(status);
    }
    if(mpu6050_ready()) {
      mpu6050_clear_ready();
      const uint8_t *buffer = mpu6050_raw_data(); //temperature available at indeces: 6 and 7
      int16_t acc_X = (int16_t)((buffer[0] << 8) | (buffer[1]));
      int16_t acc_Y = (int16_t)((buffer[2] << 8) | (buffer[3]));
      int16_t acc_Z = (int16_t)((buffer[4] << 8) | (buffer[5]));

      sum_X = sum_X + acc_X;
      sum_Y = sum_Y + acc_Y;
      sum_Z = sum_Z + acc_Z;
      i++;
    }
  }
  acc_error_X = sum_X / ACC_CALIB_SAMPLE_COUNT;
  acc_error_Y = sum_Y / ACC_CALIB_SAMPLE_COUNT;
}

void mpu6050_init(HAL_StatusTypeDef *status) {
  HAL_StatusTypeDef ret;

  ret = HAL_I2C_IsDeviceReady(&hi2c1, MPU_DEVICE_ADDRESS, 1, 100);
  if(ret == HAL_OK){
    uint8_t buffer[] = "MPU-6050 connected\n";
    CDC_Transmit_FS(buffer, sizeof(buffer));
  } else {
    uint8_t buffer1[] = "Cannot connect to MPU-6050, returning from initialization\n";
    CDC_Transmit_FS(buffer1, sizeof(buffer1));
    *status = HAL_ERROR;
    I2C_Dispatch_SetOwner(&hi2c1, I2C_OWNER_NONE);
    return;
  }

  uint8_t temp_data = 0;
  ret = HAL_I2C_Mem_Write(&hi2c1, MPU_DEVICE_ADDRESS, PWR_MGMT1_REG, I2C_MEMADD_SIZE_8BIT, &temp_data, sizeof(temp_data), 100);
  if(ret == HAL_OK){
    uint8_t buffer[] = "PWR_MGMT1 configured\n";
    CDC_Transmit_FS(buffer, sizeof(buffer));
  } else {
    uint8_t buffer1[] = "Cannot configure PWR_MGMT1, returning form initialization\n";
    CDC_Transmit_FS(buffer1, sizeof(buffer1));
    *status = HAL_ERROR;
    I2C_Dispatch_SetOwner(&hi2c1, I2C_OWNER_NONE);
    return;
  }

  temp_data = FS_GYRO_500;
  ret = HAL_I2C_Mem_Write(&hi2c1, MPU_DEVICE_ADDRESS, REG_CONFIG_GYRO, I2C_MEMADD_SIZE_8BIT, &temp_data, sizeof(temp_data), 100);
  if(ret == HAL_OK){
    uint8_t buffer[] = "Gyro configured\n";
    CDC_Transmit_FS(buffer, sizeof(buffer));
  } else {
    uint8_t buffer1[] = "Cannot configure gyro, returning form initialization\n";
    CDC_Transmit_FS(buffer1, sizeof(buffer1));
    *status = HAL_ERROR;
    I2C_Dispatch_SetOwner(&hi2c1, I2C_OWNER_NONE);
    return;
  }

  temp_data = FS_ACC_8;
  ret = HAL_I2C_Mem_Write(&hi2c1, MPU_DEVICE_ADDRESS, REG_CONFIG_ACC, I2C_MEMADD_SIZE_8BIT, &temp_data, sizeof(temp_data), 100);
  if(ret == HAL_OK){
    uint8_t buffer[] = "Accelerometer configured\n";
    CDC_Transmit_FS(buffer, sizeof(buffer));
  } else {
    uint8_t buffer1[] = "Cannot configure ACCELEROMETER, returning form initialization\n";
    CDC_Transmit_FS(buffer1, sizeof(buffer1));
    *status = HAL_ERROR;
    I2C_Dispatch_SetOwner(&hi2c1, I2C_OWNER_NONE);
    return;
  }

  temp_data = 0x03;
  ret = HAL_I2C_Mem_Write(&hi2c1, MPU_DEVICE_ADDRESS, REG_CONFIG, I2C_MEMADD_SIZE_8BIT, &temp_data, sizeof(temp_data), 100);
  if(ret == HAL_OK){
    uint8_t buffer[] = "LPF configured\n";
    CDC_Transmit_FS(buffer, sizeof(buffer));
  } else {
    uint8_t buffer1[] = "Cannot configure LPF, returning form initialization\n";
    CDC_Transmit_FS(buffer1, sizeof(buffer1));
    *status = HAL_ERROR;
    I2C_Dispatch_SetOwner(&hi2c1, I2C_OWNER_NONE);
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
  gyro_calibrate(status);
  acc_calibrate(status); 
  
  *status = HAL_OK;
}



void MPU6050_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != hi2c1.Instance) return;
    mpu_busy = 0;
    mpu_read_ready = 1;
}

void MPU6050_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != hi2c1.Instance) return;
    mpu_busy = 0;
    mpu_read_ready = 0;
}

uint8_t mpu6050_read_INT_status(HAL_StatusTypeDef *status) {
  uint8_t data;
  *status = HAL_I2C_Mem_Read(&hi2c1, MPU_DEVICE_ADDRESS , REG_INT_STATUS, I2C_MEMADD_SIZE_8BIT, &data, sizeof(data), 10); 
  return data;
}

uint8_t mpu6050_read_raw(HAL_StatusTypeDef *status, Imu* imu) {
  uint8_t new_sample = 0;

  if(mpu6050_ready()) {
    mpu6050_clear_ready();
    const uint8_t *buffer = mpu6050_raw_data(); //temperature available at indeces: 6 and 7
    imu->acc_x_raw = (int16_t)((buffer[0] << 8) | (buffer[1])) - acc_error_X;
    imu->acc_y_raw = (int16_t)((buffer[2] << 8) | (buffer[3])) - acc_error_Y; 
    imu->acc_z_raw = (int16_t)((buffer[4] << 8) | (buffer[5]));

    imu->gyro_x_raw = ((int16_t)((buffer[8] << 8) | (buffer[9]))) - gyro_error_X;
    imu->gyro_y_raw = ((int16_t)((buffer[10] << 8) | (buffer[11]))) - gyro_error_Y;
    imu->gyro_z_raw = ((int16_t)((buffer[12] << 8) | (buffer[13]))) - gyro_error_Z;
    new_sample = 1;
  }

  /*
   * Keep MPU reads pipelined: after consuming a ready sample (or at startup),
   * immediately queue the next DMA read when the bus is free.
   */
  if(!mpu6050_is_busy() && !mpu6050_ready()) {
    mpu6050_read_DMA_start(status);
  }

  return new_sample;
}

void mpu6050_test(HAL_StatusTypeDef *status) {
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
    acc_X = (int16_t)((buffer[0] << 8) | (buffer[1])) - acc_error_X;
    acc_Y = (int16_t)((buffer[2] << 8) | (buffer[3])) - acc_error_Y;
    acc_Z = (int16_t)((buffer[4] << 8) | (buffer[5]));

    gyro_X = ((int16_t)((buffer[8] << 8) | (buffer[9]))) - gyro_error_X;
    gyro_Y = ((int16_t)((buffer[10] << 8) | (buffer[11]))) - gyro_error_Y;
    gyro_Z = ((int16_t)((buffer[12] << 8) | (buffer[13]))) - gyro_error_Z;
    
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

void mpu6050_test_plot(HAL_StatusTypeDef *status, Imu *imu_test) {
  
  mpu6050_read_raw(status, imu_test);

  

  char cdc_buf[128];
  int len = snprintf(cdc_buf, sizeof(cdc_buf), ">x_acc:%d,y_acc:%d,z_acc:%d,x_gyro:%d,y_gyro:%d,z_gyro:%d\r\n", (int16_t) imu_test->acc_x_raw, (int16_t)imu_test->acc_y_raw, (int16_t)imu_test->acc_z_raw, (int16_t)imu_test->gyro_x_raw, (int16_t)imu_test->gyro_y_raw, (int16_t)imu_test->gyro_z_raw);
  
  if(len > 0){
    if (len > sizeof(cdc_buf)) {
      len = sizeof(cdc_buf);  
    }
    while (CDC_Transmit_FS((uint8_t*)cdc_buf, len) == USBD_BUSY) {
      HAL_Delay(1);
    }
  }
  HAL_Delay(10);
}



