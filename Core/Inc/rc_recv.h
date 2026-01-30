#ifndef RC_RECV_H_
#define RC_RECV_H_



#include <stdbool.h>
#define IBUS_MAX_LENGTH 32
#define IBUS_DMA_BUF_SIZE 128
#define CHANNEL_COUNT 6

typedef struct {
    uint16_t roll;
    uint16_t pitch;
    uint16_t throttle;
    uint16_t yaw;
    bool armed;
    uint16_t aux2;

} Rc_Input;

const uint8_t *read_rc_recv(HAL_StatusTypeDef *status);
uint8_t get_ibus_frame_ready(void);
void clear_ibus_frame_ready(void);
void ibus_dma_poll(void);
void ibus_init();
uint8_t ibus_read_channels(uint16_t *out);
uint8_t ibus_read_channels_struct(Rc_Input* rc_input);
uint16_t* print_channels();

#endif