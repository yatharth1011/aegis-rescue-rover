/*
 * Minimal VL53L0X driver for the ESP-IDF i2c_master API.
 *
 * Register sequence follows ST's API defaults as distilled in Pololu's
 * vl53l0x-arduino library: data init, SPAD selection from NVM, default
 * tuning, VHV + phase reference calibration, then continuous back-to-back
 * ranging. The measurement timing budget is left at the post-init default
 * (about 33 ms, so about 30 Hz per sensor).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

typedef struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
    uint8_t addr;
    uint8_t stop_variable;
    uint16_t last_mm;
    bool ok;
} vl53l0x_t;

/* Talk to a sensor at the default address 0x29 and move it to new_addr.
 * Only one un-addressed sensor may be out of reset (XSHUT high) at a time. */
esp_err_t vl53l0x_init(vl53l0x_t *s, i2c_master_bus_handle_t bus, uint8_t new_addr);
esp_err_t vl53l0x_start_continuous(vl53l0x_t *s);
#define VL53L0X_FAR_MM 2000   /* reported when nothing is within range */

/* Non-blocking: ESP_OK and mm when a new result is ready; ESP_ERR_NOT_FINISHED
 * if not; ESP_ERR_INVALID_RESPONSE when the sensor flags the range as unreliable. */
esp_err_t vl53l0x_poll_mm(vl53l0x_t *s, uint16_t *mm);
