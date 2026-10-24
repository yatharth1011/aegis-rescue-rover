/*
 * Task_Vision (core 0): receives target reports from the ESP32-CAM over UART1.
 *
 * Line protocol, ASCII, one report per camera frame:
 *     $T,<visible 0|1>,<ex>,<ey>,<area_frac>,<conf>*<XX>\n
 * ex, ey in [-1, 1] (+ex = target right of image centre), area_frac in
 * [0, 1], XX = XOR of every byte between '$' and '*', as two hex digits
 * (NMEA style). Lines with a bad checksum or out-of-range fields are dropped.
 */
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "aegis_config.h"
#include "aegis_types.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "vision";

#define CAM_UART UART_NUM_1
#define CAM_LINE_MAX 96

static bool parse_line(const char *line, vision_msg_t *out)
{
    if (line[0] != '$') return false;
    const char *star = strchr(line, '*');
    if (!star || strlen(star) < 3) return false;
    uint8_t x = 0;
    for (const char *p = line + 1; p < star; p++) x ^= (uint8_t)*p;
    if (x != (uint8_t)strtoul(star + 1, NULL, 16)) return false;

    int vis;
    float ex, ey, area, conf;
    if (sscanf(line, "$T,%d,%f,%f,%f,%f*", &vis, &ex, &ey, &area, &conf) != 5) return false;
    if (ex < -1.0f || ex > 1.0f || ey < -1.0f || ey > 1.0f || area < 0.0f || area > 1.0f) return false;
    out->t_us = esp_timer_get_time();
    out->visible = vis != 0;
    out->ex = ex;
    out->ey = ey;
    out->area_frac = area;
    out->conf = conf;
    return true;
}

void task_vision_link(void *arg)
{
    uart_config_t uc = {
        .baud_rate = CAM_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(CAM_UART, 1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(CAM_UART, &uc));
    ESP_ERROR_CHECK(uart_set_pin(CAM_UART, UART_PIN_NO_CHANGE, PIN_CAM_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    char line[CAM_LINE_MAX];
    int len = 0;
    uint32_t good = 0, bad = 0;
    for (;;) {
        uint8_t ch;
        if (uart_read_bytes(CAM_UART, &ch, 1, pdMS_TO_TICKS(1000)) != 1) continue;
        if (ch == '\n' || ch == '\r') {
            if (len > 0) {
                line[len] = '\0';
                vision_msg_t m;
                if (parse_line(line, &m)) {
                    xQueueOverwrite(q_vision, &m);
                    good++;
                } else if (++bad % 50 == 1) {
                    ESP_LOGW(TAG, "dropped malformed line (%" PRIu32 " bad / %" PRIu32 " good)", bad, good);
                }
            }
            len = 0;
        } else if (len < CAM_LINE_MAX - 1) {
            line[len++] = (char)ch;
        } else {
            len = 0;   /* overlong line: resync on next newline */
        }
    }
}
