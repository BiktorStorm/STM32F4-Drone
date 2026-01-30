#include "drone_control.h"
#include "mpu6050.h"
#include "rc_recv.h"
#include "motor_control.h"
#include "stm32f4xx_hal_def.h"
#include "math.h"
#include "stdbool.h"


typedef struct {
    float m1, m2, m3, m4; // 0..1
} motors_t;

typedef struct {
    float kp, ki, kd;

    float i;           // integrator state
    float imax;        // max abs integrator (anti-windup)

    float d_lp;        // low-passed derivative term (optional but recommended)
    float d_alpha;     // 0..1 (higher = more smoothing)

    float prev_meas;
    bool has_prev;
} pid_t;

static pid_t pid_roll  = {.kp=0.10f, .ki=0.00f, .kd=0.002f, .imax=0.3f, .d_alpha=0.9f};
static pid_t pid_pitch = {.kp=0.10f, .ki=0.00f, .kd=0.002f, .imax=0.3f, .d_alpha=0.9f};
static pid_t pid_yaw   = {.kp=0.10f, .ki=0.00f, .kd=0.000f, .imax=0.3f, .d_alpha=0.9f};

typedef struct {
    float roll;   // deg
    float pitch;  // deg
    bool  inited;
} attitude_t;

typedef struct {
    float k_roll;
    float k_pitch;
    float max_rate_dps;
} angle_loop_t;

static angle_loop_t angle_cfg = {
    .k_roll = 6.0f,
    .k_pitch = 6.0f,
    .max_rate_dps = 200.0f
};

static attitude_t att = {0};

static bool was_armed = false;


int16_t ax_imu = 0;
int16_t ay_imu = 0;
int16_t az_imu = 0;
int16_t gx_imu = 0;
int16_t gy_imu = 0;
int16_t gz_imu = 0;

extern volatile float acc_scale;
extern int16_t gyro_error_X;
extern int16_t gyro_error_Y;
extern int16_t gyro_error_Z;

float norm_1000_2000(int us) {
    // maps 1000..2000 to -1..+1 (with 1500 as 0)
    float x = ((float)us - 1500.0f) / 500.0f;
    if (x < -1) x = -1;
    if (x >  1) x =  1;
    return x;
}

static inline float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void pid_reset(pid_t *pid)
{
    pid->i = 0.0f;
    pid->d_lp = 0.0f;
    pid->prev_meas = 0.0f;
    pid->has_prev = false;
}

motors_t mix_quad_x(float T, float R, float P, float Y, bool armed)
{
    motors_t m;

    const float IDLE = 0.05f;

    if (!armed) {
        m.m1 = m.m2 = m.m3 = m.m4 = 0.0f;
        return m;
    }

    if (T < IDLE) T = IDLE;

    // raw mix (your motor order FR,RR,RL,FL)
    m.m1 = T + P - R + Y;
    m.m2 = T - P - R - Y;
    m.m3 = T - P + R + Y;
    m.m4 = T + P + R - Y;

    // desaturate + clamp
    float mn = fminf(fminf(m.m1,m.m2), fminf(m.m3,m.m4));
    float mx = fmaxf(fmaxf(m.m1,m.m2), fmaxf(m.m3,m.m4));

    if (mn < 0.0f) { m.m1 -= mn; m.m2 -= mn; m.m3 -= mn; m.m4 -= mn; }
    mx = fmaxf(fmaxf(m.m1,m.m2), fmaxf(m.m3,m.m4));
    if (mx > 1.0f) {
        float s = 1.0f / mx;
        m.m1 *= s; m.m2 *= s; m.m3 *= s; m.m4 *= s;
    }

    m.m1 = clampf(m.m1, 0.0f, 1.0f);
    m.m2 = clampf(m.m2, 0.0f, 1.0f);
    m.m3 = clampf(m.m3, 0.0f, 1.0f);
    m.m4 = clampf(m.m4, 0.0f, 1.0f);

    return m;
}

float pid_rate_update(pid_t *pid,
                      float sp_dps, float meas_dps,
                      float dt,
                      int allow_integrator)
{
    if (dt <= 0.0f) return 0.0f;

    float err = sp_dps - meas_dps;

    // --- P ---
    float p_term = pid->kp * err;

    // --- I ---
    if (allow_integrator) {
        pid->i += pid->ki * err * dt;
        pid->i = clampf(pid->i, -pid->imax, pid->imax);
    } else {
        pid->i = 0.0f;
    }

    // --- D (on measurement) ---
    float d_term = 0.0f;
    if (pid->kd > 0.0f) {
        float d_meas = 0.0f;

        if (pid->has_prev) {
            d_meas = (meas_dps - pid->prev_meas) / dt;
        }

        pid->prev_meas = meas_dps;
        pid->has_prev = true;

        pid->d_lp = pid->d_alpha * pid->d_lp +
                    (1.0f - pid->d_alpha) * d_meas;

        d_term = -pid->kd * pid->d_lp;
    }

    return p_term + pid->i + d_term;
}

void angle_loop_update(float roll_sp_deg, float pitch_sp_deg,
                       float roll_deg, float pitch_deg,
                       float *p_sp_dps, float *q_sp_dps)
{
    float e_roll  = roll_sp_deg  - roll_deg;
    float e_pitch = pitch_sp_deg - pitch_deg;

    float p_cmd = angle_cfg.k_roll  * e_roll;
    float q_cmd = angle_cfg.k_pitch * e_pitch;

    *p_sp_dps = clampf(p_cmd, -angle_cfg.max_rate_dps, angle_cfg.max_rate_dps);
    *q_sp_dps = clampf(q_cmd, -angle_cfg.max_rate_dps, angle_cfg.max_rate_dps);
}

static inline float inv_sqrt(float x) {
    return 1.0f / sqrtf(x);
}

void attitude_update_complementary(attitude_t *att,
                                   float p, float q, float r,   // deg/s (r unused here)
                                   float ax, float ay, float az, // in g (or any units)
                                   float dt)
{
    // 1) normalize accel (helps when units aren’t perfect)
    float norm2 = ax*ax + ay*ay + az*az;
    if (norm2 < 1e-6f) return; // avoid divide by zero

    float invn = inv_sqrt(norm2);
    ax *= invn; ay *= invn; az *= invn;

    // 2) accel-based angles
    float roll_acc  = atan2f(ay, az) * RAD2DEG;
    float pitch_acc = atan2f(-ax, sqrtf(ay*ay + az*az)) * RAD2DEG;

    // 3) init on first run (prevents weird startup transient)
    if (!att->inited) {
        att->roll = roll_acc;
        att->pitch = pitch_acc;
        att->inited = true;
        return;
    }

    // 4) complementary blend
    const float tau = 0.5f;                  // seconds (tune 0.3..1.0)
    float alpha = tau / (tau + dt);

    att->roll  = alpha * (att->roll  + p * dt) + (1.0f - alpha) * roll_acc;
    att->pitch = alpha * (att->pitch + q * dt) + (1.0f - alpha) * pitch_acc;
}



void control_update(float dt, HAL_StatusTypeDef* status) {
    bool new_sample = false;
    ibus_dma_poll();
    uint16_t channels[CHANNEL_COUNT];
    ibus_read_channels(channels);
    
    

    if(!mpu6050_is_busy() && !mpu6050_ready()) {
        mpu6050_read_DMA_start(status);
    }
    if(mpu6050_ready()) {
        mpu6050_clear_ready();
        const uint8_t *buffer = mpu6050_raw_data(); //temperature available at indeces: 6 and 7
        ax_imu = ((int16_t)((buffer[0] << 8) | (buffer[1])));
        ay_imu = ((int16_t)((buffer[2] << 8) | (buffer[3])));
        az_imu = ((int16_t)((buffer[4] << 8) | (buffer[5])));

        gx_imu = ((int16_t)((buffer[8] << 8) | (buffer[9]))) - gyro_error_X;
        gy_imu = ((int16_t)((buffer[10] << 8) | (buffer[11]))) - gyro_error_Y;
        gz_imu = ((int16_t)((buffer[12] << 8) | (buffer[13]))) - gyro_error_Z;

        new_sample = true;
    }
    
    if (!new_sample) {
        esc_set_us(1000, TIM_CHANNEL_1); // this should be romevd later this is just for safety 
        esc_set_us(1000, TIM_CHANNEL_2); // this should be romevd later this is just for safety
        esc_set_us(1000, TIM_CHANNEL_3); // this should be romevd later this is just for safety
        esc_set_us(1000, TIM_CHANNEL_4); // this should be romevd later this is just for safety
        return; // or just skip the rest that depends on IMU
    }

    // --- Axis transform: IMU -> Body ---
    // IMU:  X left,  Y back, Z up
    // Body: X fwd,   Y right, Z up
    int16_t ax_body_raw = -ay_imu;
    int16_t ay_body_raw = -ax_imu;
    int16_t az_body_raw =  az_imu;

    int16_t gx_body_raw = -gy_imu;   // p (roll rate about body X)
    int16_t gy_body_raw = -gx_imu;   // q (pitch rate about body Y)
    int16_t gz_body_raw =  gz_imu;   // r (yaw rate about body Z)
    
    float ax_g = (float)(ax_body_raw * acc_scale) / ACC_LSB_PER_G;
    float ay_g = (float)(ay_body_raw * acc_scale) / ACC_LSB_PER_G;
    float az_g = (float)(az_body_raw * acc_scale) / ACC_LSB_PER_G;
    
    float p_dps = (float)gx_body_raw / GYRO_LSB_PER_DPS;
    float q_dps = (float)gy_body_raw / GYRO_LSB_PER_DPS;
    float r_dps = (float)gz_body_raw / GYRO_LSB_PER_DPS;

    attitude_update_complementary(&att, p_dps, q_dps, r_dps, ax_g, ay_g, az_g, dt);

    float roll_deg  = att.roll;
    float pitch_deg = att.pitch;

    // Example stick inputs in [-1, +1]
    float roll_in  = norm_1000_2000(channels[1-1]);//stick_roll;
    float pitch_in = norm_1000_2000(channels[2-1]);;//stick_pitch;
    float yaw_in   = norm_1000_2000(channels[4-1]);;//stick_yaw;
    float throttle = ((float)channels[3-1] - 1000.0f) / 1000.0f;
    bool armed = channels[5-1] > 1500;
    if (!armed && was_armed) {
    // Just DISARMED → reset everything
        pid_reset(&pid_roll);
        pid_reset(&pid_pitch);
        pid_reset(&pid_yaw);

        att.inited = false;   // optional but recommended
    }

    was_armed = armed;

    const float max_angle_deg = 25.0f;
    const float max_yaw_rate_dps = 150.0f;

    float roll_sp_deg  = roll_in  * max_angle_deg;
    float pitch_sp_deg = pitch_in * max_angle_deg;

    float p_sp_dps, q_sp_dps;
    angle_loop_update(roll_sp_deg, pitch_sp_deg, roll_deg, pitch_deg, &p_sp_dps, &q_sp_dps);

    float r_sp_dps = yaw_in * max_yaw_rate_dps;

    float R = 0.0f;
    float P = 0.0f;
    float Y = 0.0f;
    if (armed) {    
        int allow_i = channels[5-1] > 1500; // simple gating
        
        R = pid_rate_update(&pid_roll,  p_sp_dps, p_dps, dt, allow_i);
        P = pid_rate_update(&pid_pitch, q_sp_dps, q_dps, dt, allow_i);
        Y = pid_rate_update(&pid_yaw,   r_sp_dps, r_dps, dt, allow_i);
    }


    float T = clampf(throttle, 0.0f, 1.0f);

    // optional: clamp PID outputs so they don’t dominate
    const float UMAX = 0.5f;
    R = clampf(R, -UMAX, UMAX);
    P = clampf(P, -UMAX, UMAX);
    Y = clampf(Y, -UMAX, UMAX);

    motors_t mot = mix_quad_x(T, R, P, Y, armed);

    // Convert 0..1 -> 1000..2000 us
    uint16_t us1 = 1000 + (uint16_t)(mot.m1 * 1000.0f);
    uint16_t us2 = 1000 + (uint16_t)(mot.m2 * 1000.0f);
    uint16_t us3 = 1000 + (uint16_t)(mot.m3 * 1000.0f);
    uint16_t us4 = 1000 + (uint16_t)(mot.m4 * 1000.0f);

    // --- HARD SAFETY CAP (testing) ---
    if (us1 > ESC_TEST_MAX_US) us1 = ESC_TEST_MAX_US;
    if (us2 > ESC_TEST_MAX_US) us2 = ESC_TEST_MAX_US;
    if (us3 > ESC_TEST_MAX_US) us3 = ESC_TEST_MAX_US;
    if (us4 > ESC_TEST_MAX_US) us4 = ESC_TEST_MAX_US;

    // Optional: idle floor when armed (your mixer already enforces IDLE, but this is extra safety)
    // if (armed) {
    //     if (us1 < ESC_IDLE_US) us1 = ESC_IDLE_US;
    //     if (us2 < ESC_IDLE_US) us2 = ESC_IDLE_US;
    //     if (us3 < ESC_IDLE_US) us3 = ESC_IDLE_US;
    //     if (us4 < ESC_IDLE_US) us4 = ESC_IDLE_US;
    // }

    esc_set_us(us1, TIM_CHANNEL_1);
    esc_set_us(us2, TIM_CHANNEL_2);
    esc_set_us(us3, TIM_CHANNEL_3);
    esc_set_us(us4, TIM_CHANNEL_4);
    // 1) read IMU
    // 2) axis transform
    // 3) attitude estimate
    // 4) angle loop
    // 5) rate PID
    // 6) mixer
    // 7) send ESC outputs
}

