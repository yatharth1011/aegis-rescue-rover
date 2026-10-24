#include "servo.h"

#include "aegis_config.h"
#include "driver/ledc.h"
#include "esp_check.h"

static const char *TAG = "servo";

#define SERVO_HZ 50
#define SERVO_RES LEDC_TIMER_14_BIT
#define SERVO_PERIOD_US (1000000 / SERVO_HZ)

esp_err_t servo_init(void)
{
    ledc_timer_config_t tc = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = SERVO_RES,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = SERVO_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&tc), TAG, "timer");
    ledc_channel_config_t cc = {
        .gpio_num = PIN_SERVO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&cc), TAG, "channel");
    servo_write_us(SERVO_US_OPEN);
    return ESP_OK;
}

void servo_write_us(uint32_t us)
{
    if (us < SERVO_US_MIN) us = SERVO_US_MIN;
    if (us > SERVO_US_MAX) us = SERVO_US_MAX;
    uint32_t duty = (uint32_t)(((uint64_t)us << 14) / SERVO_PERIOD_US);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}
