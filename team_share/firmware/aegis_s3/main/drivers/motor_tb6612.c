#include "motor_tb6612.h"

#include <math.h>

#include "aegis_config.h"
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_check.h"

static const char *TAG = "motor";

#define TIMER_RES_HZ 10000000
#define PERIOD_TICKS (TIMER_RES_HZ / MOTOR_PWM_HZ)   /* 500 ticks at 20 kHz */

typedef struct {
    mcpwm_cmpr_handle_t cmp;
    mcpwm_gen_handle_t gen_in1, gen_in2;
    bool invert;
    int dir;   /* +1, -1, 0: which generator currently carries the PWM */
} motor_t;

static motor_t s_m[WHEEL_COUNT];

static esp_err_t setup_gen_actions(mcpwm_gen_handle_t gen, mcpwm_cmpr_handle_t cmp)
{
    /* active-low pulse: low from period start until compare, then high */
    ESP_RETURN_ON_ERROR(mcpwm_generator_set_action_on_timer_event(gen,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW)),
        TAG, "tez action");
    ESP_RETURN_ON_ERROR(mcpwm_generator_set_action_on_compare_event(gen,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmp, MCPWM_GEN_ACTION_HIGH)),
        TAG, "cmp action");
    return ESP_OK;
}

esp_err_t motors_init(void)
{
    const int in1[WHEEL_COUNT] = PIN_MOTOR_IN1;
    const int in2[WHEEL_COUNT] = PIN_MOTOR_IN2;
    const bool inv[WHEEL_COUNT] = MOTOR_INVERT;

    gpio_config_t stby = {.pin_bit_mask = 1ULL << PIN_MOTOR_STBY, .mode = GPIO_MODE_OUTPUT};
    ESP_RETURN_ON_ERROR(gpio_config(&stby), TAG, "stby");
    gpio_set_level(PIN_MOTOR_STBY, 0);

    mcpwm_timer_handle_t timers[2];
    for (int g = 0; g < 2; g++) {
        mcpwm_timer_config_t tc = {
            .group_id = g,
            .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = TIMER_RES_HZ,
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
            .period_ticks = PERIOD_TICKS,
        };
        ESP_RETURN_ON_ERROR(mcpwm_new_timer(&tc, &timers[g]), TAG, "timer %d", g);
    }

    for (int i = 0; i < WHEEL_COUNT; i++) {
        int group = i < 3 ? 0 : 1;
        mcpwm_oper_handle_t oper;
        mcpwm_operator_config_t oc = {.group_id = group};
        ESP_RETURN_ON_ERROR(mcpwm_new_operator(&oc, &oper), TAG, "oper %d", i);
        ESP_RETURN_ON_ERROR(mcpwm_operator_connect_timer(oper, timers[group]), TAG, "connect %d", i);

        mcpwm_comparator_config_t cc = {.flags.update_cmp_on_tez = true};
        ESP_RETURN_ON_ERROR(mcpwm_new_comparator(oper, &cc, &s_m[i].cmp), TAG, "cmp %d", i);
        ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(s_m[i].cmp, 0), TAG, "cmp0 %d", i);

        mcpwm_generator_config_t g1 = {.gen_gpio_num = in1[i]};
        mcpwm_generator_config_t g2 = {.gen_gpio_num = in2[i]};
        ESP_RETURN_ON_ERROR(mcpwm_new_generator(oper, &g1, &s_m[i].gen_in1), TAG, "gen1 %d", i);
        ESP_RETURN_ON_ERROR(mcpwm_new_generator(oper, &g2, &s_m[i].gen_in2), TAG, "gen2 %d", i);
        ESP_RETURN_ON_ERROR(setup_gen_actions(s_m[i].gen_in1, s_m[i].cmp), TAG, "act1 %d", i);
        ESP_RETURN_ON_ERROR(setup_gen_actions(s_m[i].gen_in2, s_m[i].cmp), TAG, "act2 %d", i);

        /* start braked: both inputs forced high */
        mcpwm_generator_set_force_level(s_m[i].gen_in1, 1, true);
        mcpwm_generator_set_force_level(s_m[i].gen_in2, 1, true);
        s_m[i].invert = inv[i];
        s_m[i].dir = 0;
    }

    for (int g = 0; g < 2; g++) {
        ESP_RETURN_ON_ERROR(mcpwm_timer_enable(timers[g]), TAG, "enable %d", g);
        ESP_RETURN_ON_ERROR(mcpwm_timer_start_stop(timers[g], MCPWM_TIMER_START_NO_STOP), TAG, "start %d", g);
    }
    ESP_LOGI(TAG, "4 motors on MCPWM, %d Hz, %d ticks", MOTOR_PWM_HZ, PERIOD_TICKS);
    return ESP_OK;
}

void motors_set(const float duty[4])
{
    for (int i = 0; i < WHEEL_COUNT; i++) {
        motor_t *m = &s_m[i];
        float d = m->invert ? -duty[i] : duty[i];
        if (d > 1.0f) d = 1.0f;
        if (d < -1.0f) d = -1.0f;
        int dir = d > 0.002f ? 1 : (d < -0.002f ? -1 : 0);
        uint32_t ticks = (uint32_t)lroundf(fabsf(d) * PERIOD_TICKS);

        if (dir != m->dir) {
            /* release the PWM'd input first, so both are never low together */
            mcpwm_generator_set_force_level(m->gen_in1, 1, true);
            mcpwm_generator_set_force_level(m->gen_in2, 1, true);
            if (dir > 0) {
                mcpwm_generator_set_force_level(m->gen_in2, -1, true);   /* IN2 = PWM */
            } else if (dir < 0) {
                mcpwm_generator_set_force_level(m->gen_in1, -1, true);   /* IN1 = PWM */
            }
            m->dir = dir;
        }
        mcpwm_comparator_set_compare_value(m->cmp, dir ? ticks : 0);
    }
}

void motors_enable(bool on)
{
    if (!on) {
        const float zero[WHEEL_COUNT] = {0};
        motors_set(zero);
    }
    gpio_set_level(PIN_MOTOR_STBY, on ? 1 : 0);
}
