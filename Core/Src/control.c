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

//settings on maiden flight: P: 1.0, I : 0.0, D:80.0
#define P 3.5f      //7.0 very small Oscillation
#define I 0.05f     //0.1 is bad, 0.5 is good for now
#define D 25.0f     // 80 worked but 60 causes less jitter

// IMU calibration offsets (adjust these to compensate for drift)
#define ACC_X_OFFSET  0.1f  // Adjust for roll drift (west-east) pos = up west
#define ACC_Y_OFFSET  0.07f  // Adjust for pitch drift (south-north) pos = up south
#define ACC_Z_OFFSET  0.0f  // Adjust for vertical drift
#define GYRO_X_OFFSET 0.0f  // Adjust for pitch gyro bias
#define GYRO_Y_OFFSET 0.0f  // Adjust for roll gyro bias
#define GYRO_Z_OFFSET 0.0f  // Adjust for yaw gyro bias

extern double home_lat;
extern double home_long;
extern float home_elevation;

extern uint8_t control_busy;


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
    .rth     = false
};

  float pid_p_gain_roll = P;               //Gain setting for the roll P-controller
  float pid_i_gain_roll = I;              //Gain setting for the roll I-controller
  float pid_d_gain_roll = D;              //Gain setting for the roll D-controller
  int pid_max_roll = MAX_ROLL_PITCH;                    //Maximum output of the PID-controller (+/-)

  float pid_p_gain_pitch = P;  //Gain setting for the pitch P-controller.
  float pid_i_gain_pitch = I;  //Gain setting for the pitch I-controller.
  float pid_d_gain_pitch = D;  //Gain setting for the pitch D-controller.
  int pid_max_pitch = MAX_ROLL_PITCH;          //Maximum output of the PID-controller (+/-)

  float pid_p_gain_yaw = 4.0f;                //Gain setting for the pitch P-controller. //4.0
  float pid_i_gain_yaw = 0.03f;               //Gain setting for the pitch I-controller. //0.02
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

  float rth_distance_m = 0.0f;
  float rth_bearing_deg = 0.0f;
  float rth_heading_error_deg = 0.0f;
  bool rth_active = false;
  bool prev_rth = false;
  float rth_target_elevation = 0.0f;

  typedef enum {
    RTH_PHASE_CLIMB = 0,
    RTH_PHASE_NAVIGATE,
    RTH_PHASE_DESCEND
  } Rth_Phase;

  Rth_Phase rth_phase = RTH_PHASE_CLIMB;

  uint32_t mpu_dma_miss = 0;
  uint32_t mpu_dma_hit = 0;




static float wrap_180(float angle_deg) {
    while (angle_deg > 180.0f) angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

float clampf_local(float d, float min, float max) {
    const float t = d < min ? min : d;
    return t > max ? max : t;
}

void control_update(float dt, HAL_StatusTypeDef *status, Periph_status periph_status){
    uint8_t imu_new_sample = mpu6050_read_raw(status, &imu);
    ibus_read_channels_struct(&rc);
    if(rc.rth && periph_status == PERIPH_READY) {
        bmp_read(status, &bmp);
        gps_read(&gps);
        qmc_read(status, &qmc);     //might be to slow of a refresh rate for the RTH function 
    }
    if(imu_new_sample) {
        imu.acc_x = ((float)imu.acc_x_raw / ACC_SENS) * GRAVITY;
        imu.acc_y = -((float)imu.acc_y_raw / ACC_SENS) * GRAVITY;
        imu.acc_z = ((float)imu.acc_z_raw / ACC_SENS) * GRAVITY;
        imu.gyro_x = -((float)imu.gyro_x_raw / GYRO_SENS);  //joop brooking config  
        imu.gyro_y = -((float)imu.gyro_y_raw / GYRO_SENS);  //joop brooking config
        imu.gyro_z = -((float)imu.gyro_z_raw / GYRO_SENS);  //joop brooking config

        // Apply IMU calibration offsets
        imu.acc_x += ACC_X_OFFSET;
        imu.acc_y += ACC_Y_OFFSET;
        imu.acc_z += ACC_Z_OFFSET;
        imu.gyro_x += GYRO_X_OFFSET;
        imu.gyro_y += GYRO_Y_OFFSET;
        imu.gyro_z += GYRO_Z_OFFSET;
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
    }

    if(!auto_level){                                                          //If the quadcopter is not in auto-level mode
        pitch_level_adjust = 0;                                                 //Set the pitch angle correction to zero.
        roll_level_adjust = 0;                                                  //Set the roll angle correcion to zero.
    }
    
    if(rc.armed && !prev_armed) {
        bmp_update_home_elevation(status, &bmp);
        /*NEED TO UPDATE GPS HOME LOCATION */
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

    // if (!rc.armed) {
    //     esc_set_us_ALL(1000);
    //     return;
    // }
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
        if(rc.yaw > DEADBAND_UPPER)pid_yaw_setpoint = (rc.yaw - DEADBAND_UPPER)/3.0f;
        else if(rc.yaw < DEADBAND_LOWER)pid_yaw_setpoint = (rc.yaw - DEADBAND_LOWER)/3.0f;
    }

    float throttle_cmd = (float)rc.throttle;
    bool rth_ok = rc.rth &&
                  gps_coords_valid(home_lat, home_long) &&
                  gps_coords_valid(gps.latitude_deg, gps.longitude_deg) &&
                  home_elevation > LOWEST_VALID_HOME_ELEVATION_M &&
                  home_elevation < HIGHEST_VALID_HOME_ELEVATION_M;

    if (rth_ok && !prev_rth) {
        rth_phase = RTH_PHASE_CLIMB;
        rth_target_elevation = home_elevation + RTH_CLIMB_ALTITUDE_M;
    }
    prev_rth = rth_ok;

    if(rth_ok) {
        rth_distance_m = gps_distance_m(gps.latitude_deg, gps.longitude_deg, home_lat, home_long);
        rth_bearing_deg = gps_bearing_deg(gps.latitude_deg, gps.longitude_deg, home_lat, home_long);

        float heading_deg = calculate_heading_degrees(imu, qmc);
        rth_heading_error_deg = wrap_180(rth_bearing_deg - heading_deg);
        float current_elevation = bmp_get_elevation_asl_m(&bmp);
        float current_relative_alt_m = current_elevation - home_elevation;
        float altitude_hold_error_m = rth_target_elevation - current_elevation;
        float altitude_hold_correction = clampf_local(altitude_hold_error_m * RTH_ALT_HOLD_KP_US_PER_M,
                                                      -RTH_ALT_HOLD_MAX_CORR_US,
                                                      RTH_ALT_HOLD_MAX_CORR_US);
        float home_altitude_error_m = home_elevation - current_elevation;
        float home_altitude_hold_correction = clampf_local(home_altitude_error_m * RTH_ALT_HOLD_KP_US_PER_M,
                                                           -RTH_ALT_HOLD_MAX_CORR_US,
                                                           RTH_ALT_HOLD_MAX_CORR_US);

        pid_roll_setpoint = -roll_level_adjust;
        if (rth_phase == RTH_PHASE_CLIMB) {
            pid_yaw_setpoint = 0.0f;
            pid_pitch_setpoint = -pitch_level_adjust;
            throttle_cmd = clampf_local((float)rc.throttle + RTH_CLIMB_THROTTLE_BOOST, 1100.0f, 2000.0f);

            if (current_relative_alt_m >= (RTH_CLIMB_ALTITUDE_M - RTH_ALTITUDE_TOLERANCE_M)) {
                rth_phase = RTH_PHASE_NAVIGATE;
            }
        } else if (rth_phase == RTH_PHASE_NAVIGATE) {
            pid_yaw_setpoint = clampf_local(RTH_YAW_KP * rth_heading_error_deg,
                                            -RTH_MAX_YAW_RATE,
                                             RTH_MAX_YAW_RATE);

            float forward_cmd_us = 0.0f;

            if (fabsf(rth_heading_error_deg) < RTH_YAW_ALIGN_DEG) {
                if (rth_distance_m > RTH_START_BRAKE_M) {
                    forward_cmd_us = RTH_FORWARD_CMD_US;
                } else {
                    forward_cmd_us = RTH_FORWARD_SLOW_CMD_US;
                }
            } else {
                forward_cmd_us = 0.0f; // first rotate toward home
            }

            // Convert “virtual stick deflection” to your existing pitch-rate setpoint structure
            pid_pitch_setpoint = (forward_cmd_us / 3.0f) - pitch_level_adjust;      //might be a problem, maybe need to change forward_cmd_us values
            throttle_cmd = clampf_local((float)rc.throttle + altitude_hold_correction, 1100.0f, 2000.0f);

            if (rth_distance_m <= RTH_HOME_REACHED_M) {
                rth_phase = RTH_PHASE_DESCEND;
            }
        } else {
            pid_pitch_setpoint = -pitch_level_adjust;
            pid_yaw_setpoint = 0.0f;
            pid_roll_setpoint = -roll_level_adjust;

            if (current_elevation > (home_elevation + RTH_HOME_ELEVATION_TOL_M)) {
                throttle_cmd = clampf_local((float)rc.throttle - RTH_DESCEND_THROTTLE_REDUCE, 1100.0f, 2000.0f);
            } else {
                throttle_cmd = clampf_local((float)rc.throttle + home_altitude_hold_correction, 1100.0f, 2000.0f);
            }
        }
    }

    //------------------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------------------
    //calculate the PID values only on fresh IMU data
    if(imu_new_sample) {
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
    }
    //------------------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------------------
    //end PID value calculation
    
    if(rc.armed) {
        float esc_1_cmd = 0.0f;
        float esc_2_cmd = 0.0f;
        float esc_3_cmd = 0.0f;
        float esc_4_cmd = 0.0f;
        throttle_cmd = clampf_local(throttle_cmd, 1000.0f, (float)MAX_THROTTLE);
        esc_1_cmd = throttle_cmd - pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CW)
        esc_2_cmd = throttle_cmd + pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CCW)
        esc_3_cmd = throttle_cmd + pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CW)
        esc_4_cmd = throttle_cmd - pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CCW)

        // if (battery_voltage < 1240 && battery_voltage > 800){                   //Is the battery connected?
        //   esc_1 += esc_1 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-1 pulse for voltage drop.
        //   esc_2 += esc_2 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-2 pulse for voltage drop.
        //   esc_3 += esc_3 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-3 pulse for voltage drop.
        //   esc_4 += esc_4 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-4 pulse for voltage drop.
        // } 
        esc_1 = (uint16_t)clampf_local(esc_1_cmd, 1100.0f, 2000.0f);            //Keep the motors running.
        esc_2 = (uint16_t)clampf_local(esc_2_cmd, 1100.0f, 2000.0f);            //Keep the motors running.
        esc_3 = (uint16_t)clampf_local(esc_3_cmd, 1100.0f, 2000.0f);            //Keep the motors running.
        esc_4 = (uint16_t)clampf_local(esc_4_cmd, 1100.0f, 2000.0f);            //Keep the motors running.
        } else {
        esc_1 = 1000;
        esc_2 = 1000;
        esc_3 = 1000;
        esc_4 = 1000;
    }
    motors_set_us(esc_1, esc_2, esc_3, esc_4);  

}
