/*
 * A.E.G.I.S. rover firmware, ESP32-S3, ESP-IDF v5.3
 *
 *   core 1 : Task_NavControl  prio 20  100 Hz control loop (nothing else on this core)
 *   core 0 : Task_AudioDSP    prio 10  I2S DMA -> band-pass -> GCC-PHAT -> bearing
 *            Task_Vision      prio 8   ESP32-CAM UART reports
 *            Task_Telemetry   prio 3   2 Hz status log (micro-ROS publisher later)
 *            Wi-Fi / lwIP              (when enabled; ESP-IDF pins them to core 0)
 */
#include <inttypes.h>

#include "aegis_config.h"
#include "aegis_types.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "aegis";

QueueHandle_t q_doa;
QueueHandle_t q_vision;
SemaphoreHandle_t g_telem_mutex;
telemetry_t g_telem;

static const char *MISSION_NAME[] = {"IDLE", "CALIB", "SEARCH", "APPROACH", "GRASP",
                                     "RETURN", "RELEASE", "DONE", "FAULT"};

void task_telemetry(void *arg)
{
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(500));
        telemetry_t t;
        if (xSemaphoreTake(g_telem_mutex, pdMS_TO_TICKS(5)) != pdTRUE) continue;
        t = g_telem;
        xSemaphoreGive(g_telem_mutex);
        ESP_LOGI(TAG,
                 "%-8s pose (%.2f, %.2f, %4.0f deg) cmd (%.2f m/s, %.2f rad/s)%s "
                 "tof F%.2f%c L%.2f%c R%.2f%c doa %4.0f deg%c act %d/%d vbat %.2f "
                 "loop max %" PRIu32 " us, overruns %" PRIu32 " fault 0x%02" PRIx32,
                 MISSION_NAME[t.mission], t.x, t.y, t.theta * 57.2958f, t.v_cmd, t.w_cmd,
                 t.cbf_limited ? " [CBF]" : "",
                 t.tof_m[0], (t.tof_valid & 1) ? ' ' : '!', t.tof_m[1], (t.tof_valid & 2) ? ' ' : '!',
                 t.tof_m[2], (t.tof_valid & 4) ? ' ' : '!',
                 t.bearing_rad * 57.2958f, t.bearing_valid ? ' ' : '?', t.action, t.policy_src,
                 t.vbat, t.loop_max_us, t.loop_overruns, t.fault_code);
    }
}

void app_main(void)
{
    q_doa = xQueueCreate(1, sizeof(doa_msg_t));
    q_vision = xQueueCreate(1, sizeof(vision_msg_t));
    g_telem_mutex = xSemaphoreCreateMutex();
    configASSERT(q_doa && q_vision && g_telem_mutex);

    BaseType_t ok = pdPASS;
    ok &= xTaskCreatePinnedToCore(task_nav_control, "nav", 6144, NULL, PRIO_NAV, NULL, CORE_NAV);
    ok &= xTaskCreatePinnedToCore(task_audio_dsp, "audio", 4096, NULL, PRIO_AUDIO, NULL, CORE_AUDIO);
    ok &= xTaskCreatePinnedToCore(task_vision_link, "vision", 3072, NULL, PRIO_VISION, NULL, CORE_AUX);
    ok &= xTaskCreatePinnedToCore(task_telemetry, "telem", 3072, NULL, PRIO_TELEMETRY, NULL, CORE_AUX);
    configASSERT(ok == pdPASS);
    ESP_LOGI(TAG, "tasks started");
}
