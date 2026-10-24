/*
 * Quadrature decoding for four N20 Hall encoders on the four PCNT units of
 * the ESP32-S3 (x4 decoding, glitch filter, hardware overflow accumulation).
 */
#pragma once

#include <stdint.h>

#include "esp_err.h"

esp_err_t encoders_init(void);
/* Accumulated counts since boot, index order FL, RL, FR, RR. Sign is
 * corrected so that + is forward motion for every wheel. */
void encoders_read(int32_t counts[4]);
