/* Gripper servo on LEDC: 50 Hz, 14-bit, pulse width clamped to the safe range. */
#pragma once

#include <stdint.h>

#include "esp_err.h"

esp_err_t servo_init(void);
void servo_write_us(uint32_t us);
