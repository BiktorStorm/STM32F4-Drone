#include "mpu6050.h"
#include <main.h>
#include "usbd_cdc_if.h"    //Only here so that false error disappears with CDC_transmit_FS function


extern I2C_HandleTypeDef hi2c2;

void mpu6050_init(void) {
    HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&hi2c2, MPU_DEVICE_ADDRESS, 1, 100);
    if(ret == HAL_OK){
      uint8_t buffer[] = "MPU-6050 connected\n";
      CDC_Transmit_FS(buffer, sizeof(buffer));
    } else {
      uint8_t buffer1[] = "Cannot connect to MPU-6050, returning from initialization\n";
      CDC_Transmit_FS(buffer1, sizeof(buffer1));
      return;
    }

    uint8_t temp_data = FS_GYRO_500;
    ret = HAL_I2C_Mem_Write(&hi2c2, MPU_DEVICE_ADDRESS, REG_CONFIG_GYRO, I2C_MEMADD_SIZE_8BIT, &temp_data, sizeof(temp_data), 100);

    if(ret == HAL_OK){
      uint8_t buffer[] = "Gyro configured\n";
      CDC_Transmit_FS(buffer, sizeof(buffer));
    } else {
      uint8_t buffer1[] = "Cannot configure gyro, returning form initialization\n";
      CDC_Transmit_FS(buffer1, sizeof(buffer1));
    }
}