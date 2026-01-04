#include "motor_control.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "rc_recv.h"


static uint32_t dshot_dma_buf[DSHOT_DMA_LEN];
extern TIM_HandleTypeDef htim3;

void esc_set_us(uint16_t us) {
   if (us < 1000) us = 1000;
   if (us > 2000) us = 2000;
   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, us);
}

void esc_calibrate(void){
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2000);
    HAL_Delay(7000);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);
}

void motor_control_init(void) {
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}

void test_motor_channel(int channel){
  if (channel < 0 || channel >= CHANNEL_COUNT) return;
  uint16_t* channels = print_channels();
  esc_set_us(channels[channel-CHANNEL_CORRECTION]);
}

static uint16_t dshot_make_packet(uint16_t value_11bit, uint8_t telemetry)
{
  // value_11bit: 0..2047
  // telemetry: 0 or 1
  uint16_t packet = (uint16_t)((value_11bit << 1) | (telemetry & 0x1));

  // checksum over 12 bits (packet), XOR of 3 nibbles
  uint16_t csum = 0;
  uint16_t csum_data = packet;
  for (int i = 0; i < 3; i++) {
    csum ^= (csum_data & 0xF);
    csum_data >>= 4;
  }
  csum &= 0xF;

  return (uint16_t)((packet << 4) | csum);
}


static void dshot_prepare_dma(uint16_t packet16)
{
  // MSB first
  for (int i = 0; i < 16; i++) {
    uint16_t bit = (packet16 & 0x8000) ? 1u : 0u;
    dshot_dma_buf[i] = bit ? DSHOT_T1H : DSHOT_T0H;
    packet16 <<= 1;
  }

  // Idle low (CCR=0 => output low in PWM1)
  for (int i = 0; i < DSHOT_IDLE_SLOTS; i++) {
    dshot_dma_buf[16 + i] = 0u;
  }
}

void dshot_send_value(uint16_t value_11bit)
{
  uint16_t pkt = dshot_make_packet(value_11bit, 0);
  dshot_prepare_dma(pkt);

  // Always start from a clean state (prevents BUSY/partial state)
  HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);

  // Clear relevant flags
  __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);

  // Start DMA-driven PWM
  HAL_StatusTypeDef st = HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_1,
                                               dshot_dma_buf, DSHOT_DMA_LEN);
  if (st != HAL_OK) {
    return;
  }

  // Wait until DMA controller is done transferring CCR values
  while (HAL_DMA_GetState(htim3.hdma[TIM_DMA_ID_CC1]) != HAL_DMA_STATE_READY) {
    // spin (short)
  }

  // IMPORTANT: wait for the timer to complete at least one UPDATE event
  // so the last CCR value has actually been applied on the output.
  while (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE) == RESET) {
    // spin a few microseconds
  }
  __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);

  // Now it's safe to stop DMA/PWM and keep line low
  HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
}