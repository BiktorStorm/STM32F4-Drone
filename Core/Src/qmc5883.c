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
#include <limits.h>
#include <math.h>




static uint8_t qmc_raw[6];
static volatile uint8_t qmc_read_ready = 0;
static volatile uint8_t qmc_busy = 0;

static void qmc_print_i2c_error(const char *operation, HAL_StatusTypeDef status)
{
    char msg[128];
    int len = snprintf(msg, sizeof(msg),
                       "QMC %s error: status=%d i2c_error=0x%08lX\r\n",
                       operation,
                       (int)status,
                       (unsigned long)HAL_I2C_GetError(&hi2c1));

    if (len > 0) {
        if (len > (int)sizeof(msg)) {
            len = sizeof(msg);
        }
        CDC_Transmit_FS((uint8_t *)msg, len);
    }
}

Cal cal = {
    .offset_x = 0.0f,
    .offset_y = 0.0f,
    .offset_z = 0.0f,
    .scale_x = 1.0f,
    .scale_y = 1.0f,
    .scale_z = 1.0f
};

void qmc_read_DMA_start(HAL_StatusTypeDef *status) {
  if(qmc_busy || I2C_Dispatch_GetOwner(&hi2c1) != I2C_OWNER_NONE) {
    *status = HAL_BUSY;
    return;
  }
  qmc_busy = 1;
  qmc_read_ready = 0;

  I2C_Dispatch_SetOwner(&hi2c1, I2C_OWNER_QMC5883);
  
  *status = HAL_I2C_Mem_Read_DMA(&hi2c1, QMC_DEVICE_ADDRESS , QMC_BASE_ADDR, I2C_MEMADD_SIZE_8BIT, qmc_raw, QMC_RAW_LEN);
  if (*status != HAL_OK) {
    qmc_busy = 0;
    qmc_read_ready = 0;
    I2C_Dispatch_SetOwner(&hi2c1, I2C_OWNER_NONE);
  }
}

void qmc_init(HAL_StatusTypeDef *status)
{
    cal.offset_x = 0.0f;
    cal.offset_y = 0.0f;
    cal.offset_z = 0.0f;
    cal.scale_x = 1.0f;
    cal.scale_y = 1.0f;
    cal.scale_z = 1.0f;

    *status = HAL_I2C_IsDeviceReady(&hi2c1, QMC_DEVICE_ADDRESS, 3, 50);
    if (*status != HAL_OK) {
        qmc_print_i2c_error("ready", *status);
        I2C_Dispatch_SetOwner(&hi2c1, I2C_OWNER_NONE);
        return;
    }
    
    uint8_t setrst = 0x01;
    *status = HAL_I2C_Mem_Write(&hi2c1, QMC_DEVICE_ADDRESS, 0x0B, I2C_MEMADD_SIZE_8BIT, &setrst, 1, 100);
    if (*status != HAL_OK) {
        qmc_print_i2c_error("set/reset", *status);
        I2C_Dispatch_SetOwner(&hi2c1, I2C_OWNER_NONE);
        return;
    }
    
    uint8_t ctrl1 = 0x0D;   // OSR=512, RNG=8G, ODR=50Hz, MODE=continuous
    *status = HAL_I2C_Mem_Write(&hi2c1, QMC_DEVICE_ADDRESS, 0x09, I2C_MEMADD_SIZE_8BIT, &ctrl1, 1, 100);
    if (*status != HAL_OK) {
        qmc_print_i2c_error("config", *status);
        I2C_Dispatch_SetOwner(&hi2c1, I2C_OWNER_NONE);
        return;
    }
    
    qmc_read_DMA_start(status);
    if (*status != HAL_OK && *status != HAL_BUSY) {
        qmc_print_i2c_error("dma start", *status);
    }

    I2C_Dispatch_SetOwner(&hi2c1, I2C_OWNER_NONE);
}
uint8_t qmc_get_drdy(HAL_StatusTypeDef *status) {
    uint8_t drdy;
    *status = HAL_I2C_Mem_Read(&hi2c1, QMC_DEVICE_ADDRESS, 0x06, I2C_MEMADD_SIZE_8BIT, &drdy, 1, 100);    
    return drdy;
}



uint8_t qmc_read_latest(HAL_StatusTypeDef *status, Qmc *qmc) {
    HAL_StatusTypeDef local_status = HAL_OK;
    uint8_t has_new_sample = 0;

    if (qmc_read_ready) {
        qmc_read_ready = 0;
        has_new_sample = 1;

        qmc->mx_raw = (int16_t)((qmc_raw[1] << 8) | qmc_raw[0]);
        qmc->my_raw = (int16_t)((qmc_raw[3] << 8) | qmc_raw[2]);
        qmc->mz_raw = (int16_t)((qmc_raw[5] << 8) | qmc_raw[4]);

        qmc->mx = ((float) qmc->mx_raw - cal.offset_x) * cal.scale_x;
        qmc->my = ((float) qmc->my_raw - cal.offset_y) * cal.scale_y;
        qmc->mz = ((float) qmc->mz_raw - cal.offset_z) * cal.scale_z;
    }

    /*
     * Keep DMA pipelined: whenever no transfer is active, start the next one.
     * If another sensor currently owns I2C, qmc_read_DMA_start returns HAL_BUSY.
     */
    if (qmc_busy == 0) {
        qmc_read_DMA_start(&local_status);
    }

    if (status != NULL) {
        *status = local_status;
    }

    return has_new_sample;
}

void qmc_read(HAL_StatusTypeDef *status, Qmc *qmc) {
    (void)qmc_read_latest(status, qmc);
}

uint8_t qmc_calibrate(HAL_StatusTypeDef *status, uint32_t duration_ms) {
    HAL_StatusTypeDef local_status = HAL_OK;
    Qmc sample = {0};
    int16_t min_x = INT16_MAX, min_y = INT16_MAX, min_z = INT16_MAX;
    int16_t max_x = INT16_MIN, max_y = INT16_MIN, max_z = INT16_MIN;
    uint32_t start_tick = HAL_GetTick();
    uint16_t sample_count = 0;

    if (duration_ms < 2000U) {
        duration_ms = 2000U;
    }

    while ((HAL_GetTick() - start_tick) < duration_ms) {
        uint8_t has_new_sample = qmc_read_latest(&local_status, &sample);

        if (local_status != HAL_OK && local_status != HAL_BUSY) {
            if (status != NULL) {
                *status = local_status;
            }
            return 0;
        }

        if (!has_new_sample) {
            HAL_Delay(1);
            continue;
        }

        if (sample.mx_raw < min_x) min_x = sample.mx_raw;
        if (sample.mx_raw > max_x) max_x = sample.mx_raw;
        if (sample.my_raw < min_y) min_y = sample.my_raw;
        if (sample.my_raw > max_y) max_y = sample.my_raw;
        if (sample.mz_raw < min_z) min_z = sample.mz_raw;
        if (sample.mz_raw > max_z) max_z = sample.mz_raw;
        sample_count++;
    }

    if (sample_count < 20U) {
        if (status != NULL) {
            *status = HAL_TIMEOUT;
        }
        return 0;
    }

    float radius_x = ((float)(max_x - min_x)) * 0.5f;
    float radius_y = ((float)(max_y - min_y)) * 0.5f;
    float radius_z = ((float)(max_z - min_z)) * 0.5f;

    if (radius_x < 1.0f || radius_y < 1.0f || radius_z < 1.0f) {
        if (status != NULL) {
            *status = HAL_ERROR;
        }
        return 0;
    }

    float avg_radius = (radius_x + radius_y + radius_z) / 3.0f;

    cal.offset_x = ((float)(max_x + min_x)) * 0.5f;
    cal.offset_y = ((float)(max_y + min_y)) * 0.5f;
    cal.offset_z = ((float)(max_z + min_z)) * 0.5f;

    cal.scale_x = avg_radius / radius_x;
    cal.scale_y = avg_radius / radius_y;
    cal.scale_z = avg_radius / radius_z;

    if (status != NULL) {
        *status = HAL_OK;
    }
    return 1;
}

void qmc_test(void)
{
    HAL_StatusTypeDef status = HAL_OK;
    char msg[128];
    static Qmc qmc = {0};
    uint8_t has_new_sample = qmc_read_latest(&status, &qmc);

    if (status != HAL_OK && status != HAL_BUSY) {
        int len = snprintf(msg, sizeof(msg), "QMC read error: %d\r\n", (int)status);
        CDC_Transmit_FS((uint8_t *)msg, len);
        return;
    }

    if (!has_new_sample) {
        return;
    }

    int len = snprintf(msg, sizeof(msg),
                        ">MX:%d,MY:%d,MZ:%d\r\n",
                        qmc.mx_raw, qmc.my_raw, qmc.mz_raw
                        );

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
    float mx = qmc_data.mx;
    float my = qmc_data.my;
    float mz = qmc_data.mz;

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
    float declination = -15.0f;
    heading += declination;

    // Convert to degrees
    float heading_deg = heading * 180.0f / QMC_PI;

    // Wrap to 0..360
    if (heading_deg < 0.0f) {
        heading_deg += 360.0f;
    }
    if (heading_deg >= 360.0f) {
        heading_deg -= 360.0f;
    }

    return heading_deg;   
}

void qmc_heading_test_plot(HAL_StatusTypeDef *status, Imu *imu, Qmc *qmc) {
    qmc_read(status, qmc);
    HAL_Delay(50);
    mpu6050_read_raw(status, imu);
    imu->acc_x = ((float)imu->acc_x_raw / ACC_SENS) * GRAVITY;
    imu->acc_y = -((float)imu->acc_y_raw / ACC_SENS) * GRAVITY;
    imu->acc_z = ((float)imu->acc_z_raw / ACC_SENS) * GRAVITY;
    imu->gyro_x = -((float)imu->gyro_x_raw / GYRO_SENS);  //joop brooking config  
    imu->gyro_y = -((float)imu->gyro_y_raw / GYRO_SENS);  //joop brooking config
    imu->gyro_z = -((float)imu->gyro_z_raw / GYRO_SENS);  //joop brooking config

    
    HAL_Delay(50);
    int heading = (int) calculate_heading_degrees(*imu, *qmc);

    char msg[128];
    int len = snprintf(msg, sizeof(msg),
                        ">Heading:%d\r\n",
                        heading
                        );

    CDC_Transmit_FS((uint8_t *)msg, len);

    HAL_Delay(50);
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
