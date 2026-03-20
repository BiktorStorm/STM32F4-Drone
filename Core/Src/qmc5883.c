#include "qmc5883.h"
#include "bmp280.h"
#include "mpu6050.h"
#include "i2c.h"
#include "stm32f4xx_hal_def.h"
#include <main.h>
#include <stdint.h>
#include <string.h>
#include "usbd_cdc_if.h"
#include <stdio.h>




static uint8_t qmc_raw[6];
static volatile uint8_t qmc_read_ready = 0;
static volatile uint8_t qmc_busy = 0;

Cal cal = {0};


void qmc_init(HAL_StatusTypeDef *status)
{
    *status = HAL_I2C_IsDeviceReady(&hi2c1, QMC_DEVICE_ADDRESS, 3, 50);
    if (*status != HAL_OK) return;
    
    uint8_t setrst = 0x01;
    *status = HAL_I2C_Mem_Write(&hi2c1, QMC_DEVICE_ADDRESS, 0x0B, I2C_MEMADD_SIZE_8BIT, &setrst, 1, 100);
    if (*status != HAL_OK) return;
    
    uint8_t ctrl1 = 0x0D;   // OSR=512, RNG=8G, ODR=50Hz, MODE=continuous
    *status = HAL_I2C_Mem_Write(&hi2c1, QMC_DEVICE_ADDRESS, 0x09, I2C_MEMADD_SIZE_8BIT, &ctrl1, 1, 100);
}
uint8_t qmc_get_drdy(HAL_StatusTypeDef *status) {
    uint8_t drdy;
    *status = HAL_I2C_Mem_Read(&hi2c1, QMC_DEVICE_ADDRESS, 0x06, I2C_MEMADD_SIZE_8BIT, &drdy, 1, 100);    
    return drdy;
}

void qmc_read_DMA_start(HAL_StatusTypeDef *status) {
  if(qmc_busy || I2C_Dispatch_GetOwner(&hi2c1) != I2C_OWNER_NONE) {
    *status = HAL_BUSY;
    return;
  }
  qmc_busy = 1;
  qmc_read_ready = 0;

  I2C_Dispatch_SetOwner(&hi2c1, I2C_OWNER_QMC5883);
  
  *status = HAL_I2C_Mem_Read_DMA(&hi2c1, QMC_DEVICE_ADDRESS , QMC_BASE_ADDR, I2C_MEMADD_SIZE_8BIT, qmc_raw, QMC_RAW_LEN);
}

void qmc_read(HAL_StatusTypeDef *status, Qmc *qmc) {
    if(qmc_busy == 0 && qmc_read_ready == 0) {
    qmc_read_DMA_start(status);
    }

    if(qmc_read_ready) {
        qmc_read_ready = 0;
        
        qmc->mx_raw = (int16_t)((qmc_raw[1] << 8) | qmc_raw[0]);
        qmc->my_raw = (int16_t)((qmc_raw[3] << 8) | qmc_raw[2]);
        qmc->mz_raw = (int16_t)((qmc_raw[5] << 8) | qmc_raw[4]);

        qmc->mx = ((float) qmc->mx_raw - cal.offset_x) * cal.scale_x;
        qmc->my = ((float) qmc->my_raw - cal.offset_y) * cal.scale_y;
        qmc->mz = ((float) qmc->mz_raw - cal.offset_z) * cal.scale_z;

    }
}

void qmc_test(void)
{
    HAL_StatusTypeDef status;
    char msg[128];
    Qmc qmc_dummy = {0};
    Qmc *qmc = &qmc_dummy;
    qmc_read(&status, qmc);

    if (status == HAL_BUSY) {
        return;   // shared I2C bus busy, just try again next time
    }

    if (status != HAL_OK) {
        int len = snprintf(msg, sizeof(msg), "QMC read error: %d\r\n", (int)status);
        CDC_Transmit_FS((uint8_t *)msg, len);
        return;
    }

    /* Only print when a fresh DMA read has been completed and qmc struct updated */
    
    int len = snprintf(msg, sizeof(msg),
                        "QMC raw: X=%d Y=%d Z=%d | cal: X=%.2f Y=%.2f Z=%.2f\r\n",
                        qmc->mx_raw, qmc->my_raw, qmc->mz_raw,
                        qmc->mx, qmc->my, qmc->mz);

    CDC_Transmit_FS((uint8_t *)msg, len);

}

float calculate_heading_degrees(Imu imu_data, Qmc qmc_data)
{
    // Normalize accelerometer
    float a_norm = sqrtf(imu_data.acc_x * imu_data.acc_x + imu_data.acc_y * imu_data.acc_y + imu_data.acc_z * imu_data.acc_z);
    if (a_norm < 1e-6f) return 0.0f;

    float ax = imu_data.acc_x / a_norm;
    float ay = imu_data.acc_y / a_norm;
    float az = imu_data.acc_z / a_norm;

    // Roll and pitch from accelerometer
    float roll  = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

    // Magnetometer raw values
    float mx = qmc_data.mx_raw;
    float my = qmc_data.my_raw;
    float mz = qmc_data.mz_raw;

    // Tilt compensation
    float mx_comp = mx * cosf(pitch) + mz * sinf(pitch);

    float my_comp = mx * sinf(roll) * sinf(pitch)
                  + my * cosf(roll)
                  - mz * sinf(roll) * cosf(pitch);

    // Heading
    float heading = atan2f(-my_comp, mx_comp);  
    // sometimes atan2f(my_comp, mx_comp) is correct instead,
    // depending on sensor axis directions

    // Magnetic declination (example: ~5.0 degrees east)
    // Set this to 0 first, then tune later
    float declination = 0.0f;
    heading += declination;

    // Convert to degrees
    float heading_deg = heading * 180.0f / M_PI;

    // Wrap to 0..360
    if (heading_deg < 0.0f) {
        heading_deg += 360.0f;
    }
    if (heading_deg >= 360.0f) {
        heading_deg -= 360.0f;
    }

    return heading_deg;


}

void QMC5883_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    qmc_busy = 0;
    qmc_read_ready = 1;
    return;
}

void QMC5883_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    qmc_busy = 0;
    qmc_read_ready = 0;
    return;
}



