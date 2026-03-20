#ifndef QMC5883_H_
#define QMC5883_H_

#include "main.h"
#include "mpu6050.h"

#define QMC_DEVICE_ADDRESS (0x0D << 1)
#define QMC_BASE_ADDR 0x0
#define QMC_RAW_LEN 6
#define M_PI 3.14159265358979323846f

typedef struct{
    int16_t mx_raw;
    int16_t my_raw;
    int16_t mz_raw;
    float mx;
    float my;
    float mz;
}Qmc;

typedef enum {
    APPLY_CALIB,
    NO_CALIB,
} CALIB_FLAG;

typedef struct {   
    float offset_x;
    float offset_y;
    float offset_z;
    
    float scale_x;
    float scale_y;
    float scale_z;
} Cal;

extern I2C_HandleTypeDef hi2c1;

void qmc_init(HAL_StatusTypeDef *status);
uint8_t qmc_get_drdy(HAL_StatusTypeDef *status);
void qmc_read(HAL_StatusTypeDef *status, Qmc *qmc);
float calculate_heading_degrees(Imu imu_data, Qmc qmc_data);
void QMC5883_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
void QMC5883_ErrorCallback(I2C_HandleTypeDef *hi2c);
void qmc_test(void);

#endif