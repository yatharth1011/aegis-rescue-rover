#include "pcf8574.h"

#define PCF_TIMEOUT_MS 5

esp_err_t pcf8574_init(pcf8574_t *p, i2c_master_bus_handle_t bus, uint8_t addr)
{
    p->ok = false;
    i2c_device_config_t dc = {.dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = addr, .scl_speed_hz = 400000};
    esp_err_t err = i2c_master_bus_add_device(bus, &dc, &p->dev);
    if (err != ESP_OK) return err;
    const uint8_t all_inputs = 0xFF;            /* write 1s: every pin becomes a weak-pull-up input */
    err = i2c_master_transmit(p->dev, &all_inputs, 1, PCF_TIMEOUT_MS);
    p->ok = err == ESP_OK;
    return err;
}

esp_err_t pcf8574_read(pcf8574_t *p, uint8_t *pins)
{
    if (!p->ok) return ESP_ERR_INVALID_STATE;
    return i2c_master_receive(p->dev, pins, 1, PCF_TIMEOUT_MS);
}
