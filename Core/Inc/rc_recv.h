#ifndef RC_RECV_H_
#define RC_RECV_H_



#define IBUS_MAX_LENGTH 32
#define IBUS_DMA_BUF_SIZE 128
#define CHANNEL_COUNT 6

const uint8_t *read_rc_recv(HAL_StatusTypeDef *status);
uint8_t get_ibus_frame_ready(void);
void clear_ibus_frame_ready(void);
void ibus_dma_poll(void);
void ibus_init();
uint8_t ibus_read_channels(uint16_t *out);
uint16_t* print_channels();

#endif