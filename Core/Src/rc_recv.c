#include "main.h"
#include "mpu6050.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_uart.h"
#include "rc_recv.h"

extern UART_HandleTypeDef huart2;
static uint8_t ibus_frame[IBUS_MAX_LENGTH];
static uint8_t ibus_dma_buf[IBUS_DMA_BUF_SIZE];
static uint16_t ibus_dma_last_pos = 0;
static uint8_t ibus_index = 0;
static volatile uint8_t ibus_frame_ready = 0;
//static uint8_t frame_start = 0;

const uint8_t *read_rc_recv(HAL_StatusTypeDef *status) {
    uint8_t frame_start = 0;
    while(1) {
        *status = HAL_UART_Receive(&huart2, &frame_start, 1, HAL_MAX_DELAY);
        if (*status != HAL_OK) {
            ibus_frame[0] = 0xFF;
            return ibus_frame;
        }

        if (frame_start == 0x20) {
            ibus_frame[0] = 0x20;

            // Read remaining 31 bytes into frame[1..31]
            *status = HAL_UART_Receive(&huart2, &ibus_frame[1], IBUS_MAX_LENGTH - 1, HAL_MAX_DELAY);
            if (*status != HAL_OK) {
                ibus_frame[0] = 0xFF;
            }
            return ibus_frame;
        }
    }

    // for(int i = 0; i < 32; i++){
    //     *status = HAL_UART_Receive(&huart2, &frame_start, SIZE_BYTE_1, HAL_MAX_DELAY);
    //     if(frame_start == 0x20) {
    //         ibus_frame[0] = 0x20;
    //         *status = HAL_UART_Receive(&huart2, &ibus_frame[1], IBUS_MAX_LENGTH - 1, HAL_MAX_DELAY);
    //         return ibus_frame;
    //     }
    // }
    // ibus_frame[0] = 0xFF;
    // return ibus_frame;
}



void ibus_dma_start(void) {
    HAL_UART_Receive_DMA(&huart2, ibus_dma_buf, IBUS_DMA_BUF_SIZE);
}

static uint16_t ibus_dma_get_pos(void) {
    return (uint16_t) (IBUS_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx));
}

static void ibus_push_byte(uint8_t b) {
    if(ibus_index == 0) {
        if (b != 0x20) return;
        ibus_frame[ibus_index++] = b;
        return;    
    }
    ibus_frame[ibus_index++] = b;

    if(ibus_index >= IBUS_MAX_LENGTH) {
        if(ibus_frame[1] == 0x40) {
            ibus_frame_ready = 1;
        }
        ibus_index = 0;
    }
}

void ibus_dma_poll(void) {
    uint16_t pos = ibus_dma_get_pos();
    if(pos != ibus_dma_last_pos) {
        if(pos > ibus_dma_last_pos) {
            for (uint16_t i = ibus_dma_last_pos; i < pos; i++) {
                ibus_push_byte(ibus_dma_buf[i]);
            }
        } else {
            for (uint16_t i = ibus_dma_last_pos; i < IBUS_DMA_BUF_SIZE; i++)
                ibus_push_byte(ibus_dma_buf[i]);
            for (uint16_t i = 0; i < pos; i++)
                ibus_push_byte(ibus_dma_buf[i]);
        }
        ibus_dma_last_pos = pos;
    } 
}

uint8_t ibus_read_channels(uint16_t *out) {
    if (!ibus_frame_ready) return 0;
    ibus_frame_ready = 0;

    for(uint8_t ch = 0; ch < CHANNEL_COUNT; ch++) {
        uint8_t lo = ibus_frame[2 + 2*ch];
        uint8_t hi = ibus_frame[3 + 2*ch];
        out[ch] = (uint16_t) lo | ((uint16_t) hi << 8);
    }
    return 1;
}

void ibus_init() {
    ibus_dma_last_pos = 0;
    ibus_index = 0;
    ibus_frame_ready = 0;
    ibus_dma_start();
}

uint8_t get_ibus_frame_ready(void) { return ibus_frame_ready; }
void clear_ibus_frame_ready(void) { ibus_frame_ready = 0; } 