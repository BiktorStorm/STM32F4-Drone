#include "i2c.h"
#include "main.h"
#include <stdint.h>
#include "bmp280.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "usbd_cdc_if.h"

typedef struct
{
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;

    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
} BMP280_CalibData;

static uint8_t bmp_raw[6];
static BMP280_CalibData bmp280_calib;
static int32_t t_fine;

static volatile uint8_t bmp_read_ready = 0;
static volatile uint8_t bmp_busy = 0;

void bmp280_read_DMA_start(HAL_StatusTypeDef *status) {
  if(bmp_busy || I2C_Dispatch_GetOwner(&hi2c1) != I2C_OWNER_NONE) {
    *status = HAL_BUSY;
    return;
  }
  bmp_busy = 1;
  bmp_read_ready = 0;

  I2C_Dispatch_SetOwner(&hi2c1, I2C_OWNER_BMP280);
  
  *status = HAL_I2C_Mem_Read_DMA(&hi2c1, BMP_DEVICE_ADDRESS , BMP_PRESS_BASE, I2C_MEMADD_SIZE_8BIT, bmp_raw, BMP_RAW_LEN);
}


// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as global value
int32_t bmp280_compensate_T_int32(int32_t adc_T)
{
    int32_t var1, var2, T;

    var1 = ((((adc_T >> 3) - ((int32_t)bmp280_calib.dig_T1 << 1))) *
            ((int32_t)bmp280_calib.dig_T2)) >> 11;

    var2 = (((((adc_T >> 4) - ((int32_t)bmp280_calib.dig_T1)) *
             ((adc_T >> 4) - ((int32_t)bmp280_calib.dig_T1))) >> 12) *
            ((int32_t)bmp280_calib.dig_T3)) >> 14;

    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;

    return T;
}
// Returns pressure in Pa as unsigned 32 bit integer. Output value of “96386” equals 96386 Pa = 963.86 hPa
uint32_t bmp280_compensate_P_int32(int32_t adc_P)
{
    int32_t var1, var2;
    uint32_t p;

    var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)bmp280_calib.dig_P6);
    var2 = var2 + ((var1 * ((int32_t)bmp280_calib.dig_P5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)bmp280_calib.dig_P4) << 16);

    var1 = (((bmp280_calib.dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) +
           ((((int32_t)bmp280_calib.dig_P2) * var1) >> 1)) >> 18;

    var1 = ((((32768 + var1)) * ((int32_t)bmp280_calib.dig_P1)) >> 15);

    if (var1 == 0)
    {
        return 0;   // avoid division by zero
    }

    p = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 >> 12))) * 3125U;

    if (p < 0x80000000U)
    {
        p = (p << 1) / ((uint32_t)var1);
    }
    else
    {
        p = (p / (uint32_t)var1) * 2U;
    }

    var1 = (((int32_t)bmp280_calib.dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(p >> 2)) * ((int32_t)bmp280_calib.dig_P8)) >> 13;

    p = (uint32_t)((int32_t)p + ((var1 + var2 + bmp280_calib.dig_P7) >> 4));

    return p;
}


void bmp_read(HAL_StatusTypeDef *status, BMP *bmp) {
    if(bmp_busy == 0 && bmp_read_ready == 0) {
    bmp280_read_DMA_start(status);
    }

    if(bmp_read_ready) {
        bmp_read_ready = 0;
        
        int32_t adc_P = ((int32_t)bmp_raw[0] << 12) | ((int32_t)bmp_raw[1] << 4) | (bmp_raw[2] >> 4);
        int32_t adc_T = ((int32_t)bmp_raw[3] << 12) | ((int32_t)bmp_raw[4] << 4) | (bmp_raw[5] >> 4);

        uint32_t press = (uint32_t) bmp280_compensate_P_int32(adc_P);
        int32_t temp = (int32_t) bmp280_compensate_T_int32(adc_T);

        bmp->pressure = press;
        bmp->temp = temp;
    }
}

void bmp_test(HAL_StatusTypeDef *status){
    if(bmp_busy == 0 && bmp_read_ready == 0) {
    bmp280_read_DMA_start(status);
    }

    if(bmp_read_ready) {
        bmp_read_ready = 0;
    
        int32_t adc_P = ((int32_t)bmp_raw[0] << 12) | ((int32_t)bmp_raw[1] << 4) | (bmp_raw[2] >> 4);
        int32_t adc_T = ((int32_t)bmp_raw[3] << 12) | ((int32_t)bmp_raw[4] << 4) | (bmp_raw[5] >> 4);

        uint32_t press = (uint32_t) bmp280_compensate_P_int32(adc_P);
        int32_t temp = (int32_t) bmp280_compensate_T_int32(adc_T);



        if(*status == HAL_OK) {
        char cdc_buf[64];
        int len = snprintf(cdc_buf, sizeof(cdc_buf), "Pressure raw: %d Temp: %d\r\n", press, temp);
            if(len > 0){
                if (len > sizeof(cdc_buf)) {
                len = sizeof(cdc_buf);  
                }
                while (CDC_Transmit_FS((uint8_t*)cdc_buf, len) == USBD_BUSY) {
                HAL_Delay(1);
                }
            }
       }
    }
}



   // use 0xEC if SDO is tied to GND

HAL_StatusTypeDef BMP280_ReadCalibration()
{
    uint8_t calib[24];
    HAL_StatusTypeDef st;

    st = HAL_I2C_Mem_Read(&hi2c1, BMP_DEVICE_ADDRESS, 0x88, I2C_MEMADD_SIZE_8BIT, calib, 24, 100);
    if (st != HAL_OK) return st;

    bmp280_calib.dig_T1 = (uint16_t)(calib[1] << 8 | calib[0]);
    bmp280_calib.dig_T2 = (int16_t)(calib[3] << 8 | calib[2]);
    bmp280_calib.dig_T3 = (int16_t)(calib[5] << 8 | calib[4]);

    bmp280_calib.dig_P1 = (uint16_t)(calib[7] << 8 | calib[6]);
    bmp280_calib.dig_P2 = (int16_t)(calib[9] << 8 | calib[8]);
    bmp280_calib.dig_P3 = (int16_t)(calib[11] << 8 | calib[10]);
    bmp280_calib.dig_P4 = (int16_t)(calib[13] << 8 | calib[12]);
    bmp280_calib.dig_P5 = (int16_t)(calib[15] << 8 | calib[14]);
    bmp280_calib.dig_P6 = (int16_t)(calib[17] << 8 | calib[16]);
    bmp280_calib.dig_P7 = (int16_t)(calib[19] << 8 | calib[18]);
    bmp280_calib.dig_P8 = (int16_t)(calib[21] << 8 | calib[20]);
    bmp280_calib.dig_P9 = (int16_t)(calib[23] << 8 | calib[22]);

    return HAL_OK;
}


void bmp_init(HAL_StatusTypeDef *status) {
    HAL_StatusTypeDef ret;
    ret = HAL_I2C_IsDeviceReady(&hi2c1, BMP_DEVICE_ADDRESS, 1 ,100);
    *status = ret;
    HAL_Delay(10);

    uint8_t ctrl_meas = (1 << 5) | (3 << 2) | 3;   // osrs_t=001, osrs_p=011, mode=11
    HAL_I2C_Mem_Write(&hi2c1, 0b11101110, 0xF4, I2C_MEMADD_SIZE_8BIT, &ctrl_meas, 1, 100);

    HAL_Delay(10);
    BMP280_ReadCalibration();
    
    return;
}

void BMP280_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    bmp_busy = 0;
    bmp_read_ready = 1;
    return;
}

void BMP280_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    bmp_busy = 0;
    bmp_read_ready = 0;
    return;
}

