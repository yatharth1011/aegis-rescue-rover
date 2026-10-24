#include "encoder_pcnt.h"

#include <stdbool.h>

#include "aegis_config.h"
#include "driver/pulse_cnt.h"
#include "esp_check.h"

static const char *TAG = "encoder";

#define PCNT_LIMIT 30000

static pcnt_unit_handle_t s_unit[WHEEL_COUNT];
static bool s_invert[WHEEL_COUNT];

esp_err_t encoders_init(void)
{
    const int pa[WHEEL_COUNT] = PIN_ENC_A;
    const int pb[WHEEL_COUNT] = PIN_ENC_B;
    const bool inv[WHEEL_COUNT] = MOTOR_INVERT;

    for (int i = 0; i < WHEEL_COUNT; i++) {
        pcnt_unit_config_t uc = {
            .low_limit = -PCNT_LIMIT,
            .high_limit = PCNT_LIMIT,
            .flags.accum_count = true,     /* driver extends to int over limits */
        };
        ESP_RETURN_ON_ERROR(pcnt_new_unit(&uc, &s_unit[i]), TAG, "unit %d", i);
        pcnt_glitch_filter_config_t fc = {.max_glitch_ns = 1000};
        ESP_RETURN_ON_ERROR(pcnt_unit_set_glitch_filter(s_unit[i], &fc), TAG, "filter %d", i);

        pcnt_chan_config_t ca = {.edge_gpio_num = pa[i], .level_gpio_num = pb[i]};
        pcnt_chan_config_t cb = {.edge_gpio_num = pb[i], .level_gpio_num = pa[i]};
        pcnt_channel_handle_t ch_a, ch_b;
        ESP_RETURN_ON_ERROR(pcnt_new_channel(s_unit[i], &ca, &ch_a), TAG, "chan a %d", i);
        ESP_RETURN_ON_ERROR(pcnt_new_channel(s_unit[i], &cb, &ch_b), TAG, "chan b %d", i);
        pcnt_channel_set_edge_action(ch_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
        pcnt_channel_set_level_action(ch_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
        pcnt_channel_set_edge_action(ch_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
        pcnt_channel_set_level_action(ch_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

        /* watch points at the limits let accum_count carry over overflow */
        ESP_RETURN_ON_ERROR(pcnt_unit_add_watch_point(s_unit[i], PCNT_LIMIT), TAG, "wp hi %d", i);
        ESP_RETURN_ON_ERROR(pcnt_unit_add_watch_point(s_unit[i], -PCNT_LIMIT), TAG, "wp lo %d", i);

        ESP_RETURN_ON_ERROR(pcnt_unit_enable(s_unit[i]), TAG, "enable %d", i);
        ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(s_unit[i]), TAG, "clear %d", i);
        ESP_RETURN_ON_ERROR(pcnt_unit_start(s_unit[i]), TAG, "start %d", i);
        s_invert[i] = inv[i];
    }
    ESP_LOGI(TAG, "4 quadrature encoders, %.0f counts per wheel rev", (double)ENC_COUNTS_PER_WHEEL_REV);
    return ESP_OK;
}

void encoders_read(int32_t counts[4])
{
    for (int i = 0; i < WHEEL_COUNT; i++) {
        int c = 0;
        pcnt_unit_get_count(s_unit[i], &c);
        counts[i] = s_invert[i] ? -c : c;
    }
}
