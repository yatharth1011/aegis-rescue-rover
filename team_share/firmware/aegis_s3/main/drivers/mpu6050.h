/*
 * MPU-6050: gyro Z (yaw rate) and accelerometer for tilt detection.
 * Configured for +-500 dps, +-2 g, 44 Hz DLPF, 1 kHz internal sample rate.
 */
#pragma once

#include <stdbool.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

typedef struct {
    i2c_master_dev_handle_t dev;
    float gz_bias;         /* rad/s */
} mpu6050_t;

typedef struct {
    float ax, ay, az;      /* g */
    float gz;              /* rad/s, bias removed, + = counter-clockwise (left turn) */
} mpu6050_sample_t;

esp_err_t mpu6050_init(mpu6050_t *m, i2c_master_bus_handle_t bus, uint8_t addr);
/* Average n samples while the robot is still. Fails if motion is detected. */
esp_err_t mpu6050_calibrate_gyro(mpu6050_t *m, int n);
esp_err_t mpu6050_read(mpu6050_t *m, mpu6050_sample_t *s);
