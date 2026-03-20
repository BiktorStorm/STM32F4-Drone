#include "control.h"
#include "bmp280.h"
#include "gps.h"
#include "qmc5883.h"
#include "mpu6050.h"
#include "rc_recv.h"
#include "motor_control.h"
#include "stm32f4xx_hal_def.h"
#include "math.h"
#include "stdbool.h"

#define P 0.4f
#define I 0.006f
#define D 2

extern double home_lat;
extern double home_long;

Imu imu = {0};
BMP bmp = {0};
Qmc qmc = {0};
Gps_Data gps = {0};

Rc_Input rc = {
    .roll     = 1500,
    .pitch    = 1500,
    .yaw      = 1500,
    .throttle = 1000,
    .armed    = false,
    .aux2     = 1000
};

  float pid_p_gain_roll = P;               //Gain setting for the roll P-controller
  float pid_i_gain_roll = I;              //Gain setting for the roll I-controller
  float pid_d_gain_roll = D;              //Gain setting for the roll D-controller
  int pid_max_roll = MAX_ROLL_PITCH;                    //Maximum output of the PID-controller (+/-)

  float pid_p_gain_pitch = P;  //Gain setting for the pitch P-controller.
  float pid_i_gain_pitch = I;  //Gain setting for the pitch I-controller.
  float pid_d_gain_pitch = D;  //Gain setting for the pitch D-controller.
  int pid_max_pitch = MAX_ROLL_PITCH;          //Maximum output of the PID-controller (+/-)

  float pid_p_gain_yaw = 2.0f;                //Gain setting for the pitch P-controller. //4.0
  float pid_i_gain_yaw = 0.02f;               //Gain setting for the pitch I-controller. //0.02
  float pid_d_gain_yaw = 0.0f;                //Gain setting for the pitch D-controller.
  int pid_max_yaw = 400;          
  float pid_i_mem_roll = 0.0f, pid_roll_setpoint = 0.0f, gyro_roll_input = 0.0f, pid_output_roll = 0.0f, pid_last_roll_d_error = 0.0f;
  float pid_i_mem_pitch = 0.0f, pid_pitch_setpoint = 0.0f, gyro_pitch_input = 0.0f, pid_output_pitch = 0.0f, pid_last_pitch_d_error = 0.0f;
  float pid_i_mem_yaw = 0.0f, pid_yaw_setpoint = 0.0f, gyro_yaw_input = 0.0f, pid_output_yaw = 0.0f, pid_last_yaw_d_error = 0.0f;
  float angle_roll_acc = 0.0f, angle_pitch_acc = 0.0f, angle_pitch = 0.0f, angle_roll = 0.0f;           //Maximum output of the PID-controller (+/-)
  float roll_level_adjust = 0.0f, pitch_level_adjust = 0.0f;
  bool auto_level = true;
  float acc_total_vector = 0.0f;
  float pid_error_temp = 0.0f;
  bool prev_armed = false;

  uint16_t esc_1 = 1000, esc_2 = 1000, esc_3 = 1000, esc_4 = 1000;



void control_update(float dt, HAL_StatusTypeDef *status){
    
    mpu6050_read_raw(status, &imu);
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

    angle_pitch += imu.gyro_x * dt;
    angle_roll  += imu.gyro_y  * dt;

    angle_pitch -= angle_roll * sinf(imu.gyro_z * dt * DEG2RAD);
    angle_roll  += angle_pitch * sinf(imu.gyro_z * dt * DEG2RAD);

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
        return;
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

