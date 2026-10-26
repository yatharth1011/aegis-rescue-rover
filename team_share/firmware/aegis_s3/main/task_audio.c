/*
 * Task_AudioDSP (core 0)
 *
 * I2S DMA capture of both INMP441 microphones at 48 kHz, 4th-order Butterworth
 * band-pass (300 Hz - 3.4 kHz) per channel, band-limited GCC-PHAT, sub-sample
 * peak interpolation, then a bearing published to the nav task.
 *
 * 48 kHz rather than 44.1 kHz: with a 100 mm baseline the maximum delay is
 * 14 samples instead of 13, which improves angular resolution at no cost.
 *
 * On a "listen" request from the laptop (q_listen) it also records up to
 * CLIP_MAX_S of 16 kHz mono (mean of both mics, 3:1 decimation of the raw
 * signal) plus every valid bearing in that window, and hands the clip to the
 * net task (q_clip). The laptop runs the scream classifier on it.
 *
 * Limitation: two microphones cannot tell front from back. A source behind
 * the rover reads as its mirror image in front. The nav task stores bearings
 * in the world frame, so after a turn the estimate is corrected.
 */
#include <math.h>
#include <string.h>

#include "aegis_config.h"
#include "aegis_dsp.h"
#include "aegis_types.h"
#include "driver/i2s_std.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "audio";

#define DMA_DESC 8
#define DMA_FRAMES 512

static int32_t s_raw[AUDIO_FRAME * 2];         /* interleaved L, R */
static float s_l[AUDIO_FRAME], s_r[AUDIO_FRAME];
static gcc_phat_t s_gcc;
static bandpass_t s_bp_l, s_bp_r;
static int16_t s_pcm[CLIP_MAX_N];
static audio_clip_t s_clip = {.pcm = s_pcm};

typedef struct {
    bool active;
    int target;            /* samples to record at 16 kHz */
    float acc;
    int acc_n;
} recorder_t;

static void recorder_feed(recorder_t *r, const float *l, const float *rr, int n)
{
    for (int i = 0; i < n && s_clip.n < r->target; i++) {
        r->acc += 0.5f * (l[i] + rr[i]);
        if (++r->acc_n == AUDIO_FS / CLIP_FS) {          /* 48 k -> 16 k: mean of 3 */
            float x = r->acc / r->acc_n * 32767.0f;
            s_pcm[s_clip.n++] = (int16_t)fmaxf(-32768.0f, fminf(32767.0f, x));
            r->acc = 0;
            r->acc_n = 0;
        }
    }
}

static esp_err_t i2s_init(i2s_chan_handle_t *rx)
{
    i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    cc.dma_desc_num = DMA_DESC;
    cc.dma_frame_num = DMA_FRAMES;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&cc, NULL, rx), TAG, "channel");

    i2s_std_config_t sc = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_FS),
        /* INMP441: 24-bit data, MSB first, in a 32-bit Philips slot */
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = PIN_I2S_BCLK,
            .ws = PIN_I2S_WS,
            .dout = I2S_GPIO_UNUSED,
            .din = PIN_I2S_DIN,
            .invert_flags = {0},
        },
    };
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(*rx, &sc), TAG, "std mode");
    return i2s_channel_enable(*rx);
}

static float band_energy(const float *x, int n)
{
    double s = 0;
    for (int i = 0; i < n; i++) s += (double)x[i] * x[i];
    return (float)(s / n);
}

void task_audio_dsp(void *arg)
{
    i2s_chan_handle_t rx;
    ESP_ERROR_CHECK(i2s_init(&rx));
    const float c = speed_of_sound(AUDIO_TEMP_C);
    if (!gcc_phat_init(&s_gcc, AUDIO_FS, AUDIO_FRAME, MIC_SPACING_M / c, AUDIO_F_LO, AUDIO_F_HI)) {
        ESP_LOGE(TAG, "gcc-phat init failed");
        vTaskDelete(NULL);
    }
    bandpass_init(&s_bp_l, AUDIO_FS, AUDIO_F_LO, AUDIO_F_HI);
    bandpass_init(&s_bp_r, AUDIO_FS, AUDIO_F_LO, AUDIO_F_HI);
    ESP_LOGI(TAG, "I2S %d Hz, frame %d, lag window +-%d", AUDIO_FS, AUDIO_FRAME, s_gcc.max_lag);

    noise_floor_t nf = {.floor = -1.0f};
    recorder_t rec = {0};
    uint32_t frames = 0;
    int64_t dsp_us_max = 0;

    for (;;) {
        size_t got = 0;
        esp_err_t err = i2s_channel_read(rx, s_raw, sizeof(s_raw), &got, pdMS_TO_TICKS(200));
        if (err != ESP_OK || got != sizeof(s_raw)) {
            ESP_LOGW(TAG, "i2s read %s, %u bytes", esp_err_to_name(err), (unsigned)got);
            continue;
        }
        int64_t t0 = esp_timer_get_time();

        const float scale = 1.0f / 8388608.0f;   /* 2^23 */
        for (int i = 0; i < AUDIO_FRAME; i++) {
            s_l[i] = (float)(s_raw[2 * i] >> 8) * scale;
            s_r[i] = (float)(s_raw[2 * i + 1] >> 8) * scale;
        }
        listen_req_t req;
        if (!rec.active && s_clip.n == 0 && xQueueReceive(q_listen, &req, 0) == pdTRUE) {
            rec = (recorder_t){.active = true, .target = (int)(fminf(req.dur_s, CLIP_MAX_S) * CLIP_FS)};
            s_clip.n_doa = 0;
        }
        if (rec.active) recorder_feed(&rec, s_l, s_r, AUDIO_FRAME);   /* raw signal, before the band-pass */

        bandpass_process(&s_bp_l, s_l, AUDIO_FRAME);
        bandpass_process(&s_bp_r, s_r, AUDIO_FRAME);

        float e = 0.5f * (band_energy(s_l, AUDIO_FRAME) + band_energy(s_r, AUDIO_FRAME));
        float band_db = noise_floor_update(&nf, e);

        gcc_result_t g = gcc_phat_run(&s_gcc, s_l, s_r);
        doa_msg_t m = {
            .t_us = esp_timer_get_time(),
            .bearing_rad = doa_bearing_from_lag(g.lag_samples, AUDIO_FS, MIC_SPACING_M, c),
            .lag_samples = g.lag_samples,
            .psr = g.psr,
            .band_db = band_db,
            .valid = doa_gate(g.psr, band_db, AUDIO_PSR_GATE, AUDIO_SNR_GATE_DB),
        };
        xQueueOverwrite(q_doa, &m);
        if (rec.active) {
            if (m.valid && s_clip.n_doa < CLIP_MAX_DOA) s_clip.doa_rad[s_clip.n_doa++] = m.bearing_rad;
            if (s_clip.n >= rec.target) {
                s_clip.t_end_us = m.t_us;
                if (xSemaphoreTake(g_telem_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    s_clip.est[0] = g_telem.x;
                    s_clip.est[1] = g_telem.y;
                    s_clip.est[2] = g_telem.theta;
                    xSemaphoreGive(g_telem_mutex);
                }
                audio_clip_t *p = &s_clip;
                xQueueOverwrite(q_clip, &p);
                rec.active = false;
            }
        }

        int64_t dt = esp_timer_get_time() - t0;
        if (dt > dsp_us_max) dsp_us_max = dt;
        if (++frames % 470 == 0) {   /* about every 10 s */
            ESP_LOGI(TAG, "dsp max %lld us per %d-sample frame (budget %d us)",
                     dsp_us_max, AUDIO_FRAME, (int)(1e6 * AUDIO_FRAME / AUDIO_FS));
            dsp_us_max = 0;
        }
    }
}
