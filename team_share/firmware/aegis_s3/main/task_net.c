/*
 * Task_Net (core 0): Wi-Fi station + UDP link to the laptop "edge brain".
 *
 * Laptop -> rover, UDP port CONFIG_AEGIS_UDP_PORT, JSON:
 *   {"type":"cmd","v":0.12,"w":-0.4,"door":1}     at ~10 Hz; stops after LINK_WATCHDOG_MS
 *   {"type":"listen","dur":0.8}                    record audio for the scream detector
 * Rover -> laptop, UDP port CONFIG_AEGIS_LAPTOP_PORT:
 *   {"type":"telem",...}                           20 Hz to the laptop; broadcast at 2 Hz
 *                                                  until a laptop has sent a command (discovery)
 *   "AUD1" + id, idx, total, n (uint16 LE) + n int16 samples   audio chunks
 *   {"type":"audio_meta","id":..,"n":..,"doas":[..],"est":[..]} after the last chunk
 * The format matches the messages simulation3d/brain.py consumes.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "aegis_config.h"
#include "aegis_reflex.h"
#include "aegis_types.h"
#include "cJSON.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"

static const char *TAG = "net";

#define WIFI_UP_BIT BIT0
#define AUDIO_CHUNK 700                 /* samples per packet: 1408 bytes, under one MTU */

static EventGroupHandle_t s_wifi_ev;
static int s_sock = -1;
static struct sockaddr_in s_peer;
static bool s_have_peer;
static uint16_t s_clip_id;

static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && (id == WIFI_EVENT_STA_START || id == WIFI_EVENT_STA_DISCONNECTED)) {
        xEventGroupClearBits(s_wifi_ev, WIFI_UP_BIT);
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "Wi-Fi up, rover IP " IPSTR, IP2STR(&e->ip_info.ip));
        xEventGroupSetBits(s_wifi_ev, WIFI_UP_BIT);
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
    esp_netif_t *netif = esp_netif_create_default_wifi_sta();
    esp_netif_set_hostname(netif, "aegis-rover");
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
    esp_wifi_set_ps(WIFI_PS_NONE);              /* no power save: ~5 ms instead of ~100 ms latency */
    ESP_LOGI(TAG, "joining Wi-Fi \"%s\"", CONFIG_AEGIS_WIFI_SSID);
}

static void send_to(const void *buf, size_t len, bool broadcast)
{
    struct sockaddr_in to = s_peer;
    if (broadcast) {
        to.sin_family = AF_INET;
        to.sin_port = htons(CONFIG_AEGIS_LAPTOP_PORT);
        to.sin_addr.s_addr = htonl(INADDR_BROADCAST);
    }
    sendto(s_sock, buf, len, 0, (struct sockaddr *)&to, sizeof(to));
}

static void send_telemetry(bool broadcast)
{
    telemetry_t t;
    if (xSemaphoreTake(g_telem_mutex, pdMS_TO_TICKS(2)) != pdTRUE) return;
    t = g_telem;
    xSemaphoreGive(g_telem_mutex);
    static char buf[640];
    const char *tof_name[3] = {"front", "left", "right"};
    int n = snprintf(buf, sizeof(buf), "{\"type\":\"telem\",\"t\":%.3f,\"est\":[%.4f,%.4f,%.4f],\"tof\":{",
                     t.t_us / 1e6, t.x, t.y, t.theta);
    for (int i = 0; i < 3; i++) {   /* an invalid reading is reported as max range plus a flag */
        n += snprintf(buf + n, sizeof(buf) - n, "%s\"%s\":%.3f", i ? "," : "", tof_name[i],
                      (t.tof_valid & (1u << i)) ? t.tof_m[i] : 2.0f);
    }
    n += snprintf(buf + n, sizeof(buf) - n,
                  "},\"tof_ok\":%u,\"cliff\":{\"FL\":%d,\"FR\":%d,\"R\":%d,\"RL\":%d,\"RR\":%d},"
                  "\"capture\":%d,\"stalled\":%d,\"door\":%d,\"vbat\":%.2f,\"fault\":%lu,\"mode\":\"%s\","
                  "\"mission\":%d,\"ir_raw\":%u,\"edge_events\":%lu,\"stall_events\":%lu,\"cbf\":%d,"
                  "\"loop_max_us\":%lu}",
                  t.tof_valid, !!(t.cliff_mask & CLIFF_FL), !!(t.cliff_mask & CLIFF_FR), !!(t.cliff_mask & CLIFF_R),
                  !!(t.cliff_mask & CLIFF_RL), !!(t.cliff_mask & CLIFF_RR), t.capture, t.stalled, t.door_open,
                  t.vbat, (unsigned long)t.fault_code, t.edge_mode ? "edge" : "standalone", (int)t.mission,
                  t.ir_raw, (unsigned long)t.edge_events, (unsigned long)t.stall_events, t.cbf_limited,
                  (unsigned long)t.loop_max_us);
    if (n > 0 && n < (int)sizeof(buf)) send_to(buf, n, broadcast);
}

static void send_clip(const audio_clip_t *c)
{
    static uint8_t pkt[12 + 2 * AUDIO_CHUNK];
    uint16_t id = ++s_clip_id;
    uint16_t total = (uint16_t)((c->n + AUDIO_CHUNK - 1) / AUDIO_CHUNK);
    for (uint16_t k = 0; k < total; k++) {
        uint16_t n = (uint16_t)fminf(AUDIO_CHUNK, c->n - k * AUDIO_CHUNK);
        memcpy(pkt, "AUD1", 4);
        memcpy(pkt + 4, &id, 2);
        memcpy(pkt + 6, &k, 2);
        memcpy(pkt + 8, &total, 2);
        memcpy(pkt + 10, &n, 2);
        memcpy(pkt + 12, c->pcm + k * AUDIO_CHUNK, 2 * n);
        send_to(pkt, 12 + 2 * n, false);
        vTaskDelay(1);                          /* pace the burst so the AP does not drop it */
    }
    static char meta[1400];
    int m = snprintf(meta, sizeof(meta), "{\"type\":\"audio_meta\",\"id\":%u,\"n\":%d,\"fs\":%d,"
                     "\"est\":[%.4f,%.4f,%.4f],\"doas\":[", id, c->n, CLIP_FS, c->est[0], c->est[1], c->est[2]);
    for (int i = 0; i < c->n_doa && m < (int)sizeof(meta) - 16; i++) {
        m += snprintf(meta + m, sizeof(meta) - m, "%s%.4f", i ? "," : "", c->doa_rad[i]);
    }
    m += snprintf(meta + m, sizeof(meta) - m, "]}");
    send_to(meta, m, false);
    ESP_LOGI(TAG, "audio clip %u sent: %d samples, %d bearings", id, c->n, c->n_doa);
}

static void handle_packet(const char *buf, int len, const struct sockaddr_in *from)
{
    cJSON *j = cJSON_ParseWithLength(buf, len);
    if (!j) return;
    const cJSON *type = cJSON_GetObjectItem(j, "type");
    if (cJSON_IsString(type)) {
        if (!s_have_peer || s_peer.sin_addr.s_addr != from->sin_addr.s_addr) {
            s_peer = *from;
            s_peer.sin_port = htons(CONFIG_AEGIS_LAPTOP_PORT);
            s_have_peer = true;
            ESP_LOGI(TAG, "laptop at %s", inet_ntoa(from->sin_addr));
        }
        if (strcmp(type->valuestring, "cmd") == 0) {
            const cJSON *v = cJSON_GetObjectItem(j, "v"), *w = cJSON_GetObjectItem(j, "w"),
                        *d = cJSON_GetObjectItem(j, "door");
            edge_cmd_t c = {
                .t_us = esp_timer_get_time(),
                .v = cJSON_IsNumber(v) ? (float)v->valuedouble : 0.0f,
                .w = cJSON_IsNumber(w) ? (float)w->valuedouble : 0.0f,
                .door_open = cJSON_IsNumber(d) && d->valuedouble > 0.5,
            };
            if (isfinite(c.v) && isfinite(c.w)) xQueueOverwrite(q_cmd, &c);
        } else if (strcmp(type->valuestring, "listen") == 0) {
            const cJSON *d = cJSON_GetObjectItem(j, "dur");
            listen_req_t r = {.dur_s = cJSON_IsNumber(d) ? (float)d->valuedouble : 0.8f};
            xQueueOverwrite(q_listen, &r);
        }
    }
    cJSON_Delete(j);
}

void task_net(void *arg)
{
    s_wifi_ev = xEventGroupCreate();
    wifi_start();
    xEventGroupWaitBits(s_wifi_ev, WIFI_UP_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    int yes = 1;
    setsockopt(s_sock, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes));
    struct timeval tv = {.tv_sec = 0, .tv_usec = 10000};
    setsockopt(s_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    struct sockaddr_in me = {.sin_family = AF_INET, .sin_port = htons(CONFIG_AEGIS_UDP_PORT),
                             .sin_addr.s_addr = htonl(INADDR_ANY)};
    ESP_ERROR_CHECK(bind(s_sock, (struct sockaddr *)&me, sizeof(me)) == 0 ? ESP_OK : ESP_FAIL);
    ESP_LOGI(TAG, "UDP %d up; telemetry to laptop port %d", CONFIG_AEGIS_UDP_PORT, CONFIG_AEGIS_LAPTOP_PORT);

    static char rx[512];
    int64_t next_telem = 0, next_hello = 0;
    for (;;) {
        struct sockaddr_in from;
        socklen_t fl = sizeof(from);
        int len = recvfrom(s_sock, rx, sizeof(rx), 0, (struct sockaddr *)&from, &fl);
        if (len > 0) handle_packet(rx, len, &from);

        int64_t now = esp_timer_get_time();
        if (s_have_peer && now >= next_telem) {
            send_telemetry(false);
            next_telem = now + 50000;           /* 20 Hz */
        }
        if (!s_have_peer && now >= next_hello) {
            send_telemetry(true);               /* discovery: the laptop learns our IP from this */
            next_hello = now + 500000;
        }
        audio_clip_t *clip;
        if (s_have_peer && xQueueReceive(q_clip, &clip, 0) == pdTRUE) {
            send_clip(clip);
            clip->n = 0;                        /* buffer free for the next recording */
        }
    }
}
