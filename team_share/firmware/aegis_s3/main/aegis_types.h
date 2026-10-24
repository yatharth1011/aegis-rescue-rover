/*
 * Messages passed between tasks.
 *
 *   Task_AudioDSP  --q_doa (mailbox)-->     Task_NavControl
 *   Task_Vision    --q_vision (mailbox)-->  Task_NavControl
 *   Task_NavControl --g_telem (mutex)-->    Task_Telemetry (and micro-ROS later)
 *
 * Mailboxes are length-1 queues written with xQueueOverwrite. The consumer
 * always sees the newest sample and the producer never blocks.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "aegis_mission.h"   /* mission_t */

typedef struct {
    int64_t t_us;          /* esp_timer time of the frame end */
    float bearing_rad;     /* + = source on the left */
    float lag_samples;
    float psr;
    float band_db;         /* band energy over noise floor */
    bool valid;            /* passed the PSR and SNR gates */
} doa_msg_t;

typedef struct {
    int64_t t_us;
    float ex, ey;          /* centroid error, -1..1 (+ex = target right of centre) */
    float area_frac;       /* bounding-box area / frame area */
    float conf;
    bool visible;
} vision_msg_t;


typedef struct {
    int64_t t_us;
    mission_t mission;
    float x, y, theta;
    float v_cmd, w_cmd;            /* after the safety filter */
    float wheel_w[4];              /* measured rad/s */
    float duty[4];
    float tof_m[3];                /* front, left, right */
    uint8_t tof_valid;             /* bit mask */
    float bearing_rad;
    bool bearing_valid;
    int action;                    /* last policy action, -1 none */
    int policy_src;
    bool cbf_limited;
    float vbat;
    uint32_t loop_overruns;
    uint32_t loop_max_us;
    uint32_t fault_code;
} telemetry_t;

/* Shared handles, created in app_main before any task starts */
extern QueueHandle_t q_doa;
extern QueueHandle_t q_vision;
extern SemaphoreHandle_t g_telem_mutex;
extern telemetry_t g_telem;

void task_audio_dsp(void *arg);
void task_nav_control(void *arg);
void task_vision_link(void *arg);
void task_telemetry(void *arg);
