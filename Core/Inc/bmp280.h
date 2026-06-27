#ifndef BMP280_H_
#define BMP280_H_

#include "main.h"
#include "stm32f4xx_hal_def.h"

#define BMP_DEVICE_ADDRESS 0b11101110
#define BMP_RAW_LEN 6
#define BMP_PRESS_BASE 0xF7
#define BMP_SEA_LEVEL_PRESSURE_PA 1013.25f
#define LOWEST_VALID_HOME_ELEVATION_M -500.0f
#define HIGHEST_VALID_HOME_ELEVATION_M 9000.0f



typedef struct{
    uint32_t pressure_raw;
    int32_t temp_raw;
    float pressure;
    float temp; 
} BMP;


typedef int32_t  BMP280_S32_t;
typedef uint32_t BMP280_U32_t;
extern I2C_HandleTypeDef hi2c1;

void bmp_init(HAL_StatusTypeDef *status);
void bmp_test(HAL_StatusTypeDef *status);
void bmp_read(HAL_StatusTypeDef *status, BMP *bmp);
void bmp_compute(BMP *bmp);
void bmp_update_home_elevation(HAL_StatusTypeDef *status, BMP *bmp);
float bmp_get_elevation_asl_m(const BMP *bmp);


void BMP280_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
void BMP280_ErrorCallback(I2C_HandleTypeDef *hi2c);
#endif
