#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

#include <stdint.h>

#define DSHOT_TICKS_PER_BIT   320u
#define DSHOT_ARR             (DSHOT_TICKS_PER_BIT - 1u)

// ~2/3 and ~1/3 duty
#define DSHOT_T1H             213u
#define DSHOT_T0H             107u

// Add a couple of "0 duty" slots at end to force a clean low idle gap
#define DSHOT_FRAME_BITS      16u
#define DSHOT_IDLE_SLOTS      2u
#define DSHOT_DMA_LEN         (DSHOT_FRAME_BITS + DSHOT_IDLE_SLOTS)

#define CHANNEL_CORRECTION 1

void esc_set_us(uint16_t us, uint32_t channel);
void esc_set_us_ALL(uint16_t us);
void esc_calibrate(void);
void test_motor_channel(int channel);
void motor_control_init(void);
void motors_set_us(uint16_t esc_1, uint16_t esc_2, uint16_t esc_3, uint16_t esc_4);
uint16_t u16_max(uint16_t a, uint16_t b);

#endif