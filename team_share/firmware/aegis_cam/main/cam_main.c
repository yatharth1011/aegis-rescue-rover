/*
 * A.E.G.I.S. ESP32-CAM (AI-Thinker, OV2640): camera node for the laptop edge brain.
 *
 *   http://<cam-ip>/stream    MJPEG, QVGA 320 x 240, about 10 fps (the laptop uses 8)
 *   http://<cam-ip>/capture   one JPEG (dataset capture, focus check)
 *   UDP broadcast to the laptop port every 2 s:  {"type":"cam","url":"http://<ip>/stream"}
 *
 * The frame size and the 42 deg vertical field of view match the simulator's camera
 * (simulation3d/world.py CAM), so the detector and the pixel-to-table projection carry over.
 */
#include <string.h>

#include "esp_camera.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"

static const char *TAG = "cam";

/* AI-Thinker ESP32-CAM pin map */
#define CAM_PWDN 32
#define CAM_RESET -1
#define CAM_XCLK 0
#define CAM_SIOD 26
#define CAM_SIOC 27
#define CAM_D7 35
#define CAM_D6 34
#define CAM_D5 39
#define CAM_D4 36
#define CAM_D3 21
#define CAM_D2 19
#define CAM_D1 18
#define CAM_D0 5
#define CAM_VSYNC 25
#define CAM_HREF 23
#define CAM_PCLK 22
#define FLASH_LED 4

#define CAM_FLIP 1   /* 1 if the image comes out upside down on the mast (check with /capture) */
#define WIFI_UP_BIT BIT0
#define FRAME_PERIOD_US 100000          /* 10 fps cap: keeps the 2.4 GHz channel free for the rover */

static EventGroupHandle_t s_ev;
static esp_ip4_addr_t s_ip;

static esp_err_t camera_start(void)
{
    camera_config_t c = {
        .pin_pwdn = CAM_PWDN, .pin_reset = CAM_RESET, .pin_xclk = CAM_XCLK,
        .pin_sccb_sda = CAM_SIOD, .pin_sccb_scl = CAM_SIOC,
        .pin_d7 = CAM_D7, .pin_d6 = CAM_D6, .pin_d5 = CAM_D5, .pin_d4 = CAM_D4,
        .pin_d3 = CAM_D3, .pin_d2 = CAM_D2, .pin_d1 = CAM_D1, .pin_d0 = CAM_D0,
        .pin_vsync = CAM_VSYNC, .pin_href = CAM_HREF, .pin_pclk = CAM_PCLK,
        .xclk_freq_hz = 20000000,
        .ledc_timer = LEDC_TIMER_0, .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_JPEG,
        .frame_size = FRAMESIZE_QVGA,
        .jpeg_quality = 12,
        .fb_count = 2,
        .fb_location = CAMERA_FB_IN_PSRAM,
        .grab_mode = CAMERA_GRAB_LATEST,        /* always the newest frame: lowest latency */
    };
    esp_err_t err = esp_camera_init(&c);
    if (err != ESP_OK) return err;
    sensor_t *s = esp_camera_sensor_get();
    s->set_vflip(s, CAM_FLIP);
    s->set_hmirror(s, CAM_FLIP);
    return ESP_OK;
}

static esp_err_t capture_handler(httpd_req_t *req)
{
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) return httpd_resp_send_500(req);
    httpd_resp_set_type(req, "image/jpeg");
    esp_err_t err = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    return err;
}

#define BOUNDARY "aegisframe"
static esp_err_t stream_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=" BOUNDARY);
    char hdr[96];
    int64_t next = 0;
    for (;;) {
        int64_t now = esp_timer_get_time();
        if (now < next) vTaskDelay(pdMS_TO_TICKS((next - now) / 1000));
        next = esp_timer_get_time() + FRAME_PERIOD_US;
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) continue;
        int n = snprintf(hdr, sizeof(hdr), "\r\n--" BOUNDARY "\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                         (unsigned)fb->len);
        esp_err_t err = httpd_resp_send_chunk(req, hdr, n);
        if (err == ESP_OK) err = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
        esp_camera_fb_return(fb);
        if (err != ESP_OK) return err;          /* laptop disconnected */
    }
}

static void http_start(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t h;
    ESP_ERROR_CHECK(httpd_start(&h, &cfg));
    httpd_uri_t stream = {.uri = "/stream", .method = HTTP_GET, .handler = stream_handler};
    httpd_uri_t capture = {.uri = "/capture", .method = HTTP_GET, .handler = capture_handler};
    httpd_register_uri_handler(h, &stream);
    httpd_register_uri_handler(h, &capture);
}

static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && (id == WIFI_EVENT_STA_START || id == WIFI_EVENT_STA_DISCONNECTED)) {
        xEventGroupClearBits(s_ev, WIFI_UP_BIT);
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        s_ip = ((ip_event_got_ip_t *)data)->ip_info.ip;
        ESP_LOGI(TAG, "stream at http://" IPSTR "/stream", IP2STR(&s_ip));
        xEventGroupSetBits(s_ev, WIFI_UP_BIT);
    }
}

static void wifi_start(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *n = esp_netif_create_default_wifi_sta();
    esp_netif_set_hostname(n, "aegis-cam");
    wifi_init_config_t wc = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wc));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event, NULL));
    wifi_config_t cfg = {0};
    strlcpy((char *)cfg.sta.ssid, CONFIG_AEGIS_WIFI_SSID, sizeof(cfg.sta.ssid));
    strlcpy((char *)cfg.sta.password, CONFIG_AEGIS_WIFI_PASSWORD, sizeof(cfg.sta.password));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_ps(WIFI_PS_NONE);
}

void app_main(void)
{
    gpio_reset_pin(FLASH_LED);
    gpio_set_direction(FLASH_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(FLASH_LED, 0);               /* the flash LED stays off: it blinds the detector */

    s_ev = xEventGroupCreate();
    if (camera_start() != ESP_OK) {
        ESP_LOGE(TAG, "camera init failed: check the ribbon cable and the 5 V supply (>= 500 mA)");
        return;
    }
    wifi_start();
    xEventGroupWaitBits(s_ev, WIFI_UP_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    http_start();

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    int yes = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes));
    struct sockaddr_in to = {.sin_family = AF_INET, .sin_port = htons(CONFIG_AEGIS_LAPTOP_PORT),
                             .sin_addr.s_addr = htonl(INADDR_BROADCAST)};
    char msg[96];
    for (;;) {                                  /* discovery beacon for the laptop */
        int n = snprintf(msg, sizeof(msg), "{\"type\":\"cam\",\"url\":\"http://" IPSTR "/stream\"}", IP2STR(&s_ip));
        sendto(sock, msg, n, 0, (struct sockaddr *)&to, sizeof(to));
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
