/*
 * Task_NavControl (core 1, 100 Hz, highest application priority)
 *
 * Every 10 ms:
 *   encoders -> wheel speeds    IMU -> yaw rate, tilt    ToF -> ranges (non-blocking)
 *   odometry (encoder distance + gyro heading)            battery voltage (10 Hz)
 *   mission state machine -> body command (v, w)
 *   CBF safety filter -> skid-steer inverse kinematics -> 4 x wheel PI -> MCPWM
 *
 * All peripherals are initialised from this task so their interrupts are
 * allocated on core 1, next to the loop that uses them.
 */
#include <inttypes.h>
#include <math.h>
#include <string.h>

#include "aegis_config.h"
#include "aegis_control.h"
#include "aegis_mission.h"
#include "aegis_policy.h"
#include "aegis_types.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "drivers/encoder_pcnt.h"
#include "drivers/motor_tb6612.h"
#include "drivers/mpu6050.h"
#include "drivers/servo.h"
#include "drivers/vl53l0x.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"

static const char *TAG = "nav";

#define PIN_START_BUTTON GPIO_NUM_0          /* BOOT button: start / abort */
#define TOF_MAX_AGE_US 150000
#define DT_S (NAV_PERIOD_MS / 1000.0f)

/* Fault codes (bit mask, latched) */
#define FAULT_TILT      (1u << 0)
#define FAULT_VBAT      (1u << 1)
#define FAULT_IMU       (1u << 2)
#define FAULT_TOF_FRONT (1u << 3)

typedef struct {
    vl53l0x_t tof[3];
    float tof_m[3];
    int64_t tof_t[3];
    mpu6050_t imu;
    bool imu_ok;
    adc_oneshot_unit_handle_t adc;
    adc_cali_handle_t adc_cali;
    float vbat;
} hw_t;

static hw_t s_hw;
static odom_t s_odom;
static mission_ctx_t s_m;
static uint32_t s_fault;
static pi_ctrl_t s_pi[WHEEL_COUNT];
static stall_det_t s_stall = {.hold_s = STALL_HOLD_S};
static const kinematics_t s_kin = {WHEEL_RADIUS_M, TRACK_M, TRACK_SKID_GAIN};

static const mission_cfg_t MISSION_CFG = {
    .v_max = V_MAX_MPS, .w_max = W_MAX_RPS,
    .cell_m = CELL_M, .turn_rad = TURN_RAD, .turn_tol_rad = TURN_TOL_RAD,
    .listen_us = 300000,
    .doa_stale_us = DOA_STALE_MS * 1000LL, .vision_stale_us = VISION_STALE_MS * 1000LL,
    .dock_area_frac = DOCK_AREA_FRAC, .dock_ex_tol = DOCK_EX_TOL,
    .path_spacing_m = PATH_SPACING_M, .path_loop_radius_m = PATH_LOOP_RADIUS_M, .home_tol_m = HOME_TOL_M,
    .vbat_low_v = VBAT_LOW_V,
    .servo_open_us = SERVO_US_OPEN, .servo_grip_us = SERVO_US_GRIP, .grip_dwell_us = 800000,
    .cbf = {CBF_D_SAFE_M, CBF_ALPHA, V_REVERSE_MAX_MPS},
    .policy = {POLICY_BLOCK_FRONT_M, POLICY_BLOCK_SIDE_M, POLICY_AUDIO_AHEAD, POLICY_USE_TABLE},
};

/* ------------------------------------------------------------------------ */
static esp_err_t tof_bringup(i2c_master_bus_handle_t bus)
{
    gpio_config_t xs = {
        .pin_bit_mask = (1ULL << PIN_TOF_XSHUT_FRONT) | (1ULL << PIN_TOF_XSHUT_LEFT),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&xs));
    gpio_set_level(PIN_TOF_XSHUT_FRONT, 0);
    gpio_set_level(PIN_TOF_XSHUT_LEFT, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* right sensor (XSHUT tied high) is the only one awake at 0x29 now */
    esp_err_t err = vl53l0x_init(&s_hw.tof[TOF_RIGHT], bus, TOF_ADDR_RIGHT);
    gpio_set_level(PIN_TOF_XSHUT_FRONT, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    esp_err_t ef = vl53l0x_init(&s_hw.tof[TOF_FRONT], bus, TOF_ADDR_FRONT);
    gpio_set_level(PIN_TOF_XSHUT_LEFT, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    esp_err_t el = vl53l0x_init(&s_hw.tof[TOF_LEFT], bus, TOF_ADDR_LEFT);

    for (int i = 0; i < 3; i++) {
        if (s_hw.tof[i].ok) vl53l0x_start_continuous(&s_hw.tof[i]);
        s_hw.tof_t[i] = 0;
    }
    if (ef != ESP_OK) return ef;   /* the front sensor is safety-critical */
    if (err != ESP_OK || el != ESP_OK) ESP_LOGW(TAG, "side ToF missing, treated as blocked");
    return ESP_OK;
}

static void adc_bringup(void)
{
    adc_oneshot_unit_init_cfg_t uc = {.unit_id = ADC_UNIT_1};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&uc, &s_hw.adc));
    adc_oneshot_chan_cfg_t cc = {.atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT};
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_hw.adc, ADC_CHANNEL_0, &cc));   /* GPIO1 */
    adc_cali_curve_fitting_config_t cal = {
        .unit_id = ADC_UNIT_1, .chan = ADC_CHANNEL_0,
        .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cal, &s_hw.adc_cali) != ESP_OK) {
        s_hw.adc_cali = NULL;
        ESP_LOGW(TAG, "no ADC calibration in eFuse, battery reading uncalibrated");
    }
}

static float read_vbat(void)
{
    int raw = 0, mv = 0;
    if (adc_oneshot_read(s_hw.adc, ADC_CHANNEL_0, &raw) != ESP_OK) return s_hw.vbat;
    if (s_hw.adc_cali) {
        adc_cali_raw_to_voltage(s_hw.adc_cali, raw, &mv);
    } else {
        mv = raw * 3100 / 4095;
    }
    return mv / 1000.0f * VBAT_DIVIDER;
}

static void hw_bringup(void)
{
    ESP_ERROR_CHECK(motors_init());
    ESP_ERROR_CHECK(encoders_init());
    ESP_ERROR_CHECK(servo_init());
    adc_bringup();

    i2c_master_bus_config_t bc = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,   /* external 2.2 k on the harness as well */
    };
    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bc, &bus));

    if (tof_bringup(bus) != ESP_OK) s_fault |= FAULT_TOF_FRONT;
    s_hw.imu_ok = mpu6050_init(&s_hw.imu, bus, MPU6050_ADDR) == ESP_OK;
    if (!s_hw.imu_ok) s_fault |= FAULT_IMU;

    gpio_config_t btn = {.pin_bit_mask = 1ULL << PIN_START_BUTTON, .mode = GPIO_MODE_INPUT,
                         .pull_up_en = GPIO_PULLUP_ENABLE};
    ESP_ERROR_CHECK(gpio_config(&btn));

    for (int i = 0; i < WHEEL_COUNT; i++) {
        pi_init(&s_pi[i], PI_KP, PI_KI, 1.0f / WHEEL_W_NOLOAD, -1.0f, 1.0f);
    }
    s_hw.vbat = read_vbat();
}

/* ------------------------------------------------------------------------ */
static void tof_service(int64_t now)
{
    for (int i = 0; i < 3; i++) {
        if (!s_hw.tof[i].ok) continue;
        uint16_t mm;
        if (vl53l0x_poll_mm(&s_hw.tof[i], &mm) == ESP_OK) {
            s_hw.tof_m[i] = mm / 1000.0f;
            s_hw.tof_t[i] = now;
        }
    }
}

static bool tof_valid(int i, int64_t now)
{
    return s_hw.tof[i].ok && (now - s_hw.tof_t[i]) < TOF_MAX_AGE_US;
}

/* ------------------------------------------------------------------------ */
void task_nav_control(void *arg)
{
    hw_bringup();
    motors_enable(false);

    mission_init(&s_m, &MISSION_CFG);
    s_m.state = MISSION_CALIBRATE;
    if (s_hw.imu_ok && mpu6050_calibrate_gyro(&s_hw.imu, 500) != ESP_OK) {
        s_fault |= FAULT_IMU;
    }
    odom_reset(&s_odom, ODOM_GYRO_WEIGHT);
    s_m.state = MISSION_IDLE;
    ESP_LOGI(TAG, "ready, faults 0x%02" PRIx32 ". Press BOOT to start.", s_fault);

    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    int32_t cnt_prev[WHEEL_COUNT];
    encoders_read(cnt_prev);
    float wheel_w[WHEEL_COUNT] = {0};
    float duty[WHEEL_COUNT] = {0};
    bool btn_prev = false;
    uint32_t tick = 0, overruns = 0, loop_max_us = 0;
    const float counts_to_rad = 2.0f * (float)M_PI / ENC_COUNTS_PER_WHEEL_REV;

    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        if (!xTaskDelayUntil(&wake, pdMS_TO_TICKS(NAV_PERIOD_MS))) overruns++;
        int64_t now = esp_timer_get_time();
        esp_task_wdt_reset();

        /* --- sense ------------------------------------------------------ */
        int32_t cnt[WHEEL_COUNT];
        encoders_read(cnt);
        for (int i = 0; i < WHEEL_COUNT; i++) {
            float w_raw = (cnt[i] - cnt_prev[i]) * counts_to_rad / DT_S;
            wheel_w[i] += 0.3f * (w_raw - wheel_w[i]);    /* EMA, ~27 ms time constant */
            cnt_prev[i] = cnt[i];
        }

        mpu6050_sample_t imu = {0};
        if (s_hw.imu_ok) {
            if (mpu6050_read(&s_hw.imu, &imu) != ESP_OK) {
                s_fault |= FAULT_IMU;
            } else if (imu.az < 0.5f) {
                s_fault |= FAULT_TILT;                  /* > 60 deg tilt or flipped */
            }
        }

        tof_service(now);
        if (!s_hw.tof[TOF_FRONT].ok) s_fault |= FAULT_TOF_FRONT;

        if (tick % 10 == 0) {
            s_hw.vbat += 0.2f * (read_vbat() - s_hw.vbat);
            if (s_hw.vbat < VBAT_CUTOFF_V && tick > 100) s_fault |= FAULT_VBAT;
        }

        mission_in_t in = {.now_us = now, .vbat = s_hw.vbat, .fault = s_fault};
        doa_msg_t doa;
        if (xQueueReceive(q_doa, &doa, 0) == pdTRUE && doa.valid) {
            in.doa_new = true;
            in.doa_bearing = doa.bearing_rad;
        }
        vision_msg_t vis;
        if (xQueueReceive(q_vision, &vis, 0) == pdTRUE) {
            in.vis_new = true;
            in.vis_visible = vis.visible;
            in.vis_ex = vis.ex;
            in.vis_area = vis.area_frac;
        }

        /* --- estimate ---------------------------------------------------- */
        float wl = 0.5f * (wheel_w[WHEEL_FL] + wheel_w[WHEEL_RL]);
        float wr = 0.5f * (wheel_w[WHEEL_FR] + wheel_w[WHEEL_RR]);
        float v_meas, w_enc;
        kin_forward(&s_kin, wl, wr, &v_meas, &w_enc);
        odom_update(&s_odom, v_meas, w_enc, s_hw.imu_ok ? imu.gz : w_enc, DT_S);

        /* --- decide ------------------------------------------------------ */
        bool btn = gpio_get_level(PIN_START_BUTTON) == 0;
        in.start_pressed = btn && !btn_prev;
        btn_prev = btn;
        in.x = s_odom.x;
        in.y = s_odom.y;
        in.theta = s_odom.theta;
        for (int i = 0; i < 3; i++) {
            in.tof_m[i] = s_hw.tof_m[i];
            in.tof_valid[i] = tof_valid(i, now);
        }
        {
            float wl_r, wr_r;
            kin_inverse(&s_kin, g_telem.v_cmd, g_telem.w_cmd, &wl_r, &wr_r);   /* last tick's reference */
            const float last_ref[WHEEL_COUNT] = {wl_r, wl_r, wr_r, wr_r};
            in.stalled = stall_update(&s_stall, duty, wheel_w, last_ref, DT_S);
        }
        mission_out_t out;
        mission_step(&s_m, &in, &out);     /* includes the CBF safety filter */
        if (out.reset_odom) odom_reset(&s_odom, ODOM_GYRO_WEIGHT);
        if (out.servo_us) servo_write_us(out.servo_us);
        bool drive = out.drive;
        motors_enable(drive);

        /* --- actuate ----------------------------------------------------- */
        float wl_ref, wr_ref;
        kin_inverse(&s_kin, out.v, out.w, &wl_ref, &wr_ref);
        const float ref[WHEEL_COUNT] = {wl_ref, wl_ref, wr_ref, wr_ref};
        float kff = 12.0f / (fmaxf(s_hw.vbat, 9.0f) * WHEEL_W_NOLOAD);   /* battery-compensated */
        for (int i = 0; i < WHEEL_COUNT; i++) {
            if (!drive) {
                pi_reset(&s_pi[i]);
                duty[i] = 0;
                continue;
            }
            s_pi[i].kff = kff;
            duty[i] = pi_update(&s_pi[i], ref[i], wheel_w[i], DT_S);
        }
        motors_set(duty);

        /* --- publish ----------------------------------------------------- */
        uint32_t exec_us = (uint32_t)(esp_timer_get_time() - now);
        if (exec_us > loop_max_us) loop_max_us = exec_us;
        if (tick % 10 == 0 && xSemaphoreTake(g_telem_mutex, 0) == pdTRUE) {
            g_telem.t_us = now;
            g_telem.mission = s_m.state;
            g_telem.x = s_odom.x;
            g_telem.y = s_odom.y;
            g_telem.theta = s_odom.theta;
            g_telem.v_cmd = out.v;
            g_telem.w_cmd = out.w;
            g_telem.tof_valid = 0;
            for (int i = 0; i < 3; i++) {
                g_telem.tof_m[i] = s_hw.tof_m[i];
                if (tof_valid(i, now)) g_telem.tof_valid |= 1u << i;
            }
            for (int i = 0; i < WHEEL_COUNT; i++) {
                g_telem.wheel_w[i] = wheel_w[i];
                g_telem.duty[i] = duty[i];
            }
            g_telem.bearing_rad = wrap_pi(s_m.doa_world - s_odom.theta);
            g_telem.bearing_valid = (now - s_m.doa_t) < DOA_STALE_MS * 1000LL;
            g_telem.action = s_m.last_action;
            g_telem.policy_src = s_m.last_src;
            g_telem.cbf_limited = out.cbf_limited;
            g_telem.vbat = s_hw.vbat;
            g_telem.loop_overruns = overruns;
            g_telem.loop_max_us = loop_max_us;
            g_telem.fault_code = s_fault;
            xSemaphoreGive(g_telem_mutex);
        }
        tick++;
    }
}
