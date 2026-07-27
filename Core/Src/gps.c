
#include "main.h"
#include "stm32f4xx_hal.h"
#include "gps.h"
#include "usbd_cdc_if.h"
#include "mpu6050.h"
#include "math.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
static uint8_t gps_dma_buf[GPS_DMA_BUF_SIZE];
static uint16_t gps_dma_last_pos = 0;

static char gps_line[GPS_LINE_MAX_LEN];
static char gps_last_line[GPS_LINE_MAX_LEN];

static uint16_t gps_index = 0;
static volatile uint8_t gps_line_ready = 0;

static Gps_Data latest_gps_data;

double home_lat = 0;
double home_long = 0;

/* ---------------- Helpers ---------------- */

static uint16_t gps_dma_get_pos(void) {
    return (uint16_t)(GPS_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx));
}

static double nmea_to_decimal_degrees(const char *val, char dir) {
    if (val == NULL || val[0] == '\0') return 0.0;

    double raw = atof(val);
    int degrees = (int)(raw / 100.0);
    double minutes = raw - (degrees * 100.0);
    double decimal = degrees + (minutes / 60.0);

    if (dir == 'S' || dir == 'W') {
        decimal = -decimal;
    }
    return decimal;
}

static void gps_parse_rmc(char *line, Gps_Data *gps_data) {
    char *fields[16] = {0};
    uint8_t count = 0;

    char *token = strtok(line, ",");
    while (token != NULL && count < 16) {
        fields[count++] = token;
        token = strtok(NULL, ",");
    }

    if (count < 10) return;

    if (fields[1] && strlen(fields[1]) >= 6) {
        gps_data->hours   = (fields[1][0] - '0') * 10 + (fields[1][1] - '0');
        gps_data->minutes = (fields[1][2] - '0') * 10 + (fields[1][3] - '0');
        gps_data->seconds = (fields[1][4] - '0') * 10 + (fields[1][5] - '0');
    }

    gps_data->valid = (fields[2] && fields[2][0] == 'A') ? 1 : 0;

    if (fields[3] && fields[4] && fields[4][0] != '\0') {
        gps_data->latitude_deg = nmea_to_decimal_degrees(fields[3], fields[4][0]);
    }

    if (fields[5] && fields[6] && fields[6][0] != '\0') {
        gps_data->longitude_deg = nmea_to_decimal_degrees(fields[5], fields[6][0]);
    }

    if (fields[7] && fields[7][0] != '\0') {
        gps_data->speed_knots = (float)atof(fields[7]);
    }

    if (fields[8] && fields[8][0] != '\0') {
        gps_data->course_deg = (float)atof(fields[8]);
    }

    if (fields[9] && strlen(fields[9]) >= 6) {
        gps_data->day   = (fields[9][0] - '0') * 10 + (fields[9][1] - '0');
        gps_data->month = (fields[9][2] - '0') * 10 + (fields[9][3] - '0');
        gps_data->year  = (fields[9][4] - '0') * 10 + (fields[9][5] - '0');
    }
}

static void gps_parse_gga(char *line, Gps_Data *gps_data) {
    char *fields[16] = {0};
    uint8_t count = 0;

    char *token = strtok(line, ",");
    while (token != NULL && count < 16) {
        fields[count++] = token;
        token = strtok(NULL, ",");
    }

    if (count < 10) return;

    if (fields[6] && fields[6][0] != '\0') {
        gps_data->fix_quality = (uint8_t)atoi(fields[6]);
    }

    if (fields[7] && fields[7][0] != '\0') {
        gps_data->satellites = (uint8_t)atoi(fields[7]);
    }

    if (fields[9] && fields[9][0] != '\0') {
        gps_data->altitude_m = (float)atof(fields[9]);
    }
}

static void gps_parse_line(const char *src, Gps_Data *gps_data) {
    char line_copy[GPS_LINE_MAX_LEN];
    strncpy(line_copy, src, GPS_LINE_MAX_LEN);
    line_copy[GPS_LINE_MAX_LEN - 1] = '\0';

    if (strncmp(line_copy, "$GNRMC", 6) == 0 || strncmp(line_copy, "$GPRMC", 6) == 0) {
        gps_parse_rmc(line_copy, gps_data);
    } else if (strncmp(line_copy, "$GNGGA", 6) == 0 || strncmp(line_copy, "$GPGGA", 6) == 0) {
        gps_parse_gga(line_copy, gps_data);
    }
}

static void gps_push_byte(uint8_t b) {
    if (gps_index == 0) {
        if (b != '$') return;
        gps_line[gps_index++] = (char)b;
        return;
    }

    if (gps_index < (GPS_LINE_MAX_LEN - 1)) {
        gps_line[gps_index++] = (char)b;
    } else {
        gps_index = 0;
        return;
    }

    if (b == '\n') {
        gps_line[gps_index] = '\0';

        strncpy(gps_last_line, gps_line, GPS_LINE_MAX_LEN);
        gps_last_line[GPS_LINE_MAX_LEN - 1] = '\0';

        gps_line_ready = 1;
        gps_index = 0;
    }
}

/* ---------------- Public functions ---------------- */

void gps_dma_start(void) {
    HAL_UART_Receive_DMA(&huart1, gps_dma_buf, GPS_DMA_BUF_SIZE);
}

void gps_dma_poll(void) {
    uint16_t pos = gps_dma_get_pos();

    if (pos != gps_dma_last_pos) {
        if (pos > gps_dma_last_pos) {
            for (uint16_t i = gps_dma_last_pos; i < pos; i++) {
                gps_push_byte(gps_dma_buf[i]);
            }
        } else {
            for (uint16_t i = gps_dma_last_pos; i < GPS_DMA_BUF_SIZE; i++) {
                gps_push_byte(gps_dma_buf[i]);
            }
            for (uint16_t i = 0; i < pos; i++) {
                gps_push_byte(gps_dma_buf[i]);
            }
        }
        gps_dma_last_pos = pos;
    }
}

uint8_t gps_read_line(char *out) {
    gps_dma_poll();
    if (!gps_line_ready) return 0;

    gps_line_ready = 0;
    strcpy(out, gps_last_line);
    return 1;
}

uint8_t gps_read(Gps_Data *gps_data) {
    gps_dma_poll();
    if (!gps_line_ready) return 0;

    gps_line_ready = 0;

    gps_parse_line(gps_last_line, &latest_gps_data);
    *gps_data = latest_gps_data;

    return 1;
}

void gps_test(void) {
    Gps_Data gps;

    if (gps_read(&gps))
    {
        char msg[200];

        int len = snprintf(msg, sizeof(msg),
            "FIX:%d SAT:%d LAT:%d LON:%d ALT:%d\r\n",
            gps.fix_quality,
            gps.satellites,
            (int)gps.latitude_deg,
            (int)gps.longitude_deg,
            (int)gps.altitude_m
            
        );

        if(len > 0){
        if (len > sizeof(msg)) {
          len = sizeof(msg);  
        }
        // CDC_Transmit_FS((uint8_t*)cdc_buf, len);
        while (CDC_Transmit_FS((uint8_t*)msg, len) == USBD_BUSY) {
          HAL_Delay(1);
        }
      }
    }
}

void gps_init(void) {
    gps_dma_last_pos = 0;
    gps_index = 0;
    gps_line_ready = 0;

    memset(gps_dma_buf, 0, sizeof(gps_dma_buf));
    memset(gps_line, 0, sizeof(gps_line));
    memset(gps_last_line, 0, sizeof(gps_last_line));
    memset(&latest_gps_data, 0, sizeof(latest_gps_data));

    gps_dma_start();


    Gps_Data tmp_gps = {0};
    // while(((int) tmp_gps.latitude_deg) == 0 && ((int) tmp_gps.longitude_deg) == 0 && tmp_gps.fix_quality < 6) {
    //     gps_read( &tmp_gps);
    //     HAL_Delay(100);
    // }
    home_lat = tmp_gps.latitude_deg;
    home_long = tmp_gps.longitude_deg;

}

bool gps_coords_valid(double lat, double lon) {
    if (lat > 90.0 || lat < -90.0) return false;
    if (lon > 180.0 || lon < -180.0) return false;
    if (lat == 0.0 && lon == 0.0) return false;
    return true;
}

float gps_distance_m(double lat1_deg, double lon1_deg, double lat2_deg, double lon2_deg) {
    const double R = 6371000.0; // Earth radius in meters

    double lat1 = lat1_deg * DEG2RAD;
    double lon1 = lon1_deg * DEG2RAD;
    double lat2 = lat2_deg * DEG2RAD;
    double lon2 = lon2_deg * DEG2RAD;

    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;

    double a = sin(dlat * 0.5) * sin(dlat * 0.5) +
               cos(lat1) * cos(lat2) *
               sin(dlon * 0.5) * sin(dlon * 0.5);

    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    return (float)(R * c);
}

float gps_bearing_deg(double lat1_deg, double lon1_deg, double lat2_deg, double lon2_deg) {
    double lat1 = lat1_deg * DEG2RAD;
    double lon1 = lon1_deg * DEG2RAD;
    double lat2 = lat2_deg * DEG2RAD;
    double lon2 = lon2_deg * DEG2RAD;

    double dlon = lon2 - lon1;

    double y = sin(dlon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon);

    double bearing = atan2(y, x) * RAD2DEG;   // -180..180
    if (bearing < 0.0) bearing += 360.0;      // 0..360
    return (float)bearing;
}

double get_home_lat(void) { return home_lat; }
double get_home_long(void) { return home_long; }
uint8_t get_gps_line_ready(void) { return gps_line_ready; }
void clear_gps_line_ready(void) { gps_line_ready = 0; }