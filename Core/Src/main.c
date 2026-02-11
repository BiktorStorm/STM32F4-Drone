/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal_def.h"
#include "usbd_cdc_if.h"
#include "mpu6050.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "rc_recv.h"
#include "motor_control.h"
#include "drone_control.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PID_LOOP_HZ 500
#define DT (1.0f / (float)PID_LOOP_HZ)
#define DEADBAND_UPPER 1510
#define DEADBAND_LOWER 1490
#define MAX_THROTTLE 1400
#define LEVEL_KP  15.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
static HAL_StatusTypeDef status = HAL_OK;
volatile uint8_t pid_timer_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  Imu imu = {0};
  Rc_Input rc = {
    .roll     = 1500,
    .pitch    = 1500,
    .yaw      = 1500,
    .throttle = 1000,
    .armed    = false,
    .aux2     = 1000
};
  float pid_p_gain_roll = 0.4;               //Gain setting for the roll P-controller
  float pid_i_gain_roll = 0.005;              //Gain setting for the roll I-controller
  float pid_d_gain_roll = 2;              //Gain setting for the roll D-controller
  int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

  float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
  float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
  float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
  int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

  float pid_p_gain_yaw = 2;                //Gain setting for the pitch P-controller. //4.0
  float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
  float pid_d_gain_yaw = 0;                //Gain setting for the pitch D-controller.
  int pid_max_yaw = 400;          
  float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
  float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
  float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
  float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;           //Maximum output of the PID-controller (+/-)
  float roll_level_adjust, pitch_level_adjust;
  angle_pitch = 0;
  angle_roll = 0;
  angle_pitch_acc = 0;
  angle_roll_acc = 0;
  bool auto_level = true;
  float acc_total_vector;
  float pid_error_temp;
  bool prev_armed = false;

  uint16_t esc_1,esc_2,esc_3,esc_4;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  mpu6050_init(&status);
  	  if(status == HAL_OK){
      uint8_t succes_msg[] = "Init success\n";
      CDC_Transmit_FS(succes_msg, sizeof(succes_msg));
	  }
  motor_control_init();
  ibus_init();
  HAL_TIM_Base_Start_IT(&htim2); //the PID loop timer on 500Hz refresh rate
  
  // esc_calibrate(); 
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
    
    while(pid_timer_flag > 0) { 
      pid_timer_flag--;
      mpu6050_read_raw(&status, &imu);
      ibus_read_channels_struct(&rc);
      imu.acc_x = ((float)imu.acc_x_raw / ACC_SENS) * GRAVITY;
      imu.acc_y = -((float)imu.acc_y_raw / ACC_SENS) * GRAVITY;
      imu.acc_z = ((float)imu.acc_z_raw / ACC_SENS) * GRAVITY;
      imu.gyro_x = -((float)imu.gyro_x_raw / GYRO_SENS);  //joop brooking config  
      imu.gyro_y = -((float)imu.gyro_y_raw / GYRO_SENS);  //joop brooking config
      imu.gyro_z = -((float)imu.gyro_z_raw / GYRO_SENS);  //joop brooking config
      gyro_roll_input = (gyro_roll_input * 0.7) + (imu.gyro_y * 0.3);   //Gyro pid input is deg/sec.
      gyro_pitch_input = (gyro_pitch_input * 0.7) + (imu.gyro_x  * 0.3);//Gyro pid input is deg/sec.
      gyro_yaw_input = (gyro_yaw_input * 0.7) + (imu.gyro_z * 0.3);      //Gyro pid input is deg/sec.

      angle_pitch += imu.gyro_x * DT;
      angle_roll  += imu.gyro_y  * DT;

      angle_pitch -= angle_roll * sinf(imu.gyro_z * DT * DEG2RAD);
      angle_roll  += angle_pitch * sinf(imu.gyro_z * DT * DEG2RAD);

      acc_total_vector = sqrtf((imu.acc_x*imu.acc_x)+(imu.acc_y*imu.acc_y)+(imu.acc_z*imu.acc_z));

      if(fabsf(imu.acc_y) < acc_total_vector) {
        angle_pitch_acc = asinf(imu.acc_y/acc_total_vector) * RAD2DEG;
      }

      if(fabsf(imu.acc_x) < acc_total_vector) {
        angle_roll_acc = asinf(imu.acc_x / acc_total_vector) * RAD2DEG;
      }

      angle_pitch = angle_pitch * 0.9998f + angle_pitch_acc * 0.0002f;
      angle_roll  = angle_roll  * 0.9998f + angle_roll_acc  * 0.0002f;

      pitch_level_adjust = angle_pitch * LEVEL_KP;                                    //Calculate the pitch angle correction
      roll_level_adjust = angle_roll * LEVEL_KP;                                      //Calculate the roll angle correction

      if(!auto_level){                                                          //If the quadcopter is not in auto-level mode
        pitch_level_adjust = 0;                                                 //Set the pitch angle correction to zero.
        roll_level_adjust = 0;                                                  //Set the roll angle correcion to zero.
      }
      
      if(rc.armed && !prev_armed) {
        angle_pitch = angle_pitch_acc;                                          //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
        angle_roll = angle_roll_acc;  
        
        pid_i_mem_roll = 0.0f;
        pid_last_roll_d_error = 0.0f;
        pid_i_mem_pitch = 0.0f;
        pid_last_pitch_d_error = 0.0f;
        pid_i_mem_yaw = 0.0f;
        pid_last_yaw_d_error = 0.0f;
      }
      prev_armed = rc.armed;

      if (!rc.armed) {
        esc_set_us_ALL(1000);
        continue;
      }
      pid_roll_setpoint = 0;
      if(rc.roll > DEADBAND_UPPER)
          pid_roll_setpoint = rc.roll - DEADBAND_UPPER;
      else if(rc.roll < DEADBAND_LOWER)
          pid_roll_setpoint = rc.roll - DEADBAND_LOWER;
      pid_roll_setpoint -= roll_level_adjust;                                   //Subtract the angle correction from the standardized receiver roll input value.
      pid_roll_setpoint /= 3.0f;    
      
      pid_pitch_setpoint = 0;
      if(rc.pitch > DEADBAND_UPPER)pid_pitch_setpoint = rc.pitch - DEADBAND_UPPER;
      else if(rc.pitch < DEADBAND_LOWER)pid_pitch_setpoint = rc.pitch - DEADBAND_LOWER;
      pid_pitch_setpoint -= pitch_level_adjust;                                  //Subtract the angle correction from the standardized receiver pitch input value.
      pid_pitch_setpoint /= 3.0f;                                                 //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

      pid_yaw_setpoint = 0;
      if(rc.throttle > 1050){ //Do not yaw when turning off the motors.
        if(rc.yaw > DEADBAND_UPPER)pid_yaw_setpoint = (rc.yaw - DEADBAND_UPPER)/3.0;
        else if(rc.yaw < DEADBAND_LOWER)pid_yaw_setpoint = (rc.yaw - DEADBAND_LOWER)/3.0;
      }
      //------------------------------------------------------------------------------------------------
      //------------------------------------------------------------------------------------------------
      //calculate the PID values
      pid_error_temp = gyro_roll_input - pid_roll_setpoint;
      pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
      if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
      else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

      pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
      if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
      else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

      pid_last_roll_d_error = pid_error_temp;

      //Pitch calculations
      pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
      pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
      if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
      else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

      pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
      if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
      else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

      pid_last_pitch_d_error = pid_error_temp;

      //Yaw calculations
      pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
      pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
      if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
      else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

      pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
      if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
      else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

      pid_last_yaw_d_error = pid_error_temp;
      //------------------------------------------------------------------------------------------------
      //------------------------------------------------------------------------------------------------
      //end PID value calculation
        
      if(rc.armed) {
        if (rc.throttle > MAX_THROTTLE) rc.throttle = MAX_THROTTLE;
        esc_1 = rc.throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
        esc_2 = rc.throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
        esc_3 = rc.throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
        esc_4 = rc.throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)

        // if (battery_voltage < 1240 && battery_voltage > 800){                   //Is the battery connected?
        //   esc_1 += esc_1 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-1 pulse for voltage drop.
        //   esc_2 += esc_2 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-2 pulse for voltage drop.
        //   esc_3 += esc_3 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-3 pulse for voltage drop.
        //   esc_4 += esc_4 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-4 pulse for voltage drop.
        // } 
        if (esc_1 < 1100) esc_1 = 1100;                                         //Keep the motors running.
        if (esc_2 < 1100) esc_2 = 1100;                                         //Keep the motors running.
        if (esc_3 < 1100) esc_3 = 1100;                                         //Keep the motors running.
        if (esc_4 < 1100) esc_4 = 1100;                                         //Keep the motors running.
      } else {
        esc_1 = 1000;
        esc_2 = 1000;
        esc_3 = 1000;
        esc_4 = 1000;
      }
      motors_set_us(esc_1, esc_2, esc_3, esc_4);  
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 9599;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 95;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        pid_timer_flag++; // 500 Hz fixed dt (0.002 seconds)
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
