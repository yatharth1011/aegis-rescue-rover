/* PCF8574 8-bit quasi-bidirectional I/O expander, used as 6 inputs for the IR sensors. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

typedef struct {
    i2c_master_dev_handle_t dev;
    bool ok;
} pcf8574_t;

esp_err_t pcf8574_init(pcf8574_t *p, i2c_master_bus_handle_t bus, uint8_t addr);
/* Reads the 8 pin levels (bit n = Pn). Inputs float high through the internal current source. */
esp_err_t pcf8574_read(pcf8574_t *p, uint8_t *pins);
