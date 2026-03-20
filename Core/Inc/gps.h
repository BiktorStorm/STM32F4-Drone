#ifndef GPS_H_
#define GPS_H_

#define GPS_DMA_BUF_SIZE   1024
#define GPS_LINE_MAX_LEN   128

typedef struct {
    uint8_t valid;

    double latitude_deg;
    double longitude_deg;

    float speed_knots;
    float course_deg;

    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;

    uint8_t day;
    uint8_t month;
    uint8_t year;

    uint8_t fix_quality;
    uint8_t satellites;
    float altitude_m;
} Gps_Data;

void gps_dma_start(void);
void gps_dma_poll(void);
void gps_init(void);

uint8_t gps_read(Gps_Data *gps_data);
uint8_t gps_read_line(char *out);

uint8_t get_gps_line_ready(void);
void clear_gps_line_ready(void);

extern UART_HandleTypeDef huart1;

#endif