#include "mpu6050.h"

#include <math.h>

#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "mpu6050";

#define I2C_TIMEOUT_MS 10
#define GYRO_LSB_PER_DPS 65.5f    /* +-500 dps */
#define ACCEL_LSB_PER_G 16384.0f  /* +-2 g */
#define DEG2RAD 0.01745329252f

static esp_err_t wr8(mpu6050_t *m, uint8_t reg, uint8_t val)
{
    uint8_t b[2] = {reg, val};
    return i2c_master_transmit(m->dev, b, 2, I2C_TIMEOUT_MS);
}

esp_err_t mpu6050_init(mpu6050_t *m, i2c_master_bus_handle_t bus, uint8_t addr)
{
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 400000,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &cfg, &m->dev), TAG, "add dev");
    uint8_t reg = 0x75, who = 0;
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(m->dev, &reg, 1, &who, 1, I2C_TIMEOUT_MS), TAG, "whoami");
    ESP_RETURN_ON_FALSE(who == 0x68, ESP_ERR_NOT_FOUND, TAG, "WHO_AM_I 0x%02x", who);

    ESP_RETURN_ON_ERROR(wr8(m, 0x6B, 0x80), TAG, "reset");           /* PWR_MGMT_1 reset */
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_RETURN_ON_ERROR(wr8(m, 0x6B, 0x01), TAG, "clk");             /* PLL on gyro X */
    ESP_RETURN_ON_ERROR(wr8(m, 0x1A, 0x03), TAG, "dlpf");            /* CONFIG: 44 Hz */
    ESP_RETURN_ON_ERROR(wr8(m, 0x19, 0x00), TAG, "smplrt");          /* 1 kHz */
    ESP_RETURN_ON_ERROR(wr8(m, 0x1B, 0x08), TAG, "gyro fs");         /* +-500 dps */
    ESP_RETURN_ON_ERROR(wr8(m, 0x1C, 0x00), TAG, "accel fs");        /* +-2 g */
    m->gz_bias = 0.0f;
    return ESP_OK;
}

static esp_err_t read_raw(mpu6050_t *m, int16_t *ax, int16_t *ay, int16_t *az, int16_t *gz)
{
    uint8_t reg = 0x3B, b[14];
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(m->dev, &reg, 1, b, sizeof(b), I2C_TIMEOUT_MS), TAG, "read");
    *ax = (int16_t)((b[0] << 8) | b[1]);
    *ay = (int16_t)((b[2] << 8) | b[3]);
    *az = (int16_t)((b[4] << 8) | b[5]);
    *gz = (int16_t)((b[12] << 8) | b[13]);
    return ESP_OK;
}

esp_err_t mpu6050_calibrate_gyro(mpu6050_t *m, int n)
{
    double sum = 0, sum2 = 0;
    for (int i = 0; i < n; i++) {
        int16_t ax, ay, az, gz;
        ESP_RETURN_ON_ERROR(read_raw(m, &ax, &ay, &az, &gz), TAG, "cal read");
        double w = gz / GYRO_LSB_PER_DPS * DEG2RAD;
        sum += w;
        sum2 += w * w;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    double mean = sum / n, sd = sqrt(fmax(0.0, sum2 / n - mean * mean));
    ESP_RETURN_ON_FALSE(sd < 0.02, ESP_ERR_INVALID_STATE, TAG, "robot moved during gyro cal (sd %.4f rad/s)", sd);
    m->gz_bias = (float)mean;
    ESP_LOGI(TAG, "gyro z bias %.5f rad/s (sd %.5f)", mean, sd);
    return ESP_OK;
}

esp_err_t mpu6050_read(mpu6050_t *m, mpu6050_sample_t *s)
{
    int16_t ax, ay, az, gz;
    ESP_RETURN_ON_ERROR(read_raw(m, &ax, &ay, &az, &gz), TAG, "read");
    s->ax = ax / ACCEL_LSB_PER_G;
    s->ay = ay / ACCEL_LSB_PER_G;
    s->az = az / ACCEL_LSB_PER_G;
    s->gz = gz / GYRO_LSB_PER_DPS * DEG2RAD - m->gz_bias;
    return ESP_OK;
}
