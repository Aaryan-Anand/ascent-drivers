#include "i2c_manager.h"
#include <stdbool.h>

// Static array to track initialization status of I2C ports
static bool i2c_initialized[I2C_NUM_MAX] = {false};

esp_err_t i2c_manager_init(int sda_pin, int scl_pin, uint32_t freq_hz, i2c_port_t port) {
    // Check if already initialized
    if (i2c_initialized[port]) {
        return ESP_OK; // Already initialized, just return success
    }

    // Configure I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = freq_hz,
    };

    esp_err_t ret = i2c_param_config(port, &conf);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) {
        return ret;
    }

    i2c_initialized[port] = true;
    return ESP_OK;
}

bool i2c_manager_is_initialized(i2c_port_t port) {
    if (port >= I2C_NUM_MAX) {
        return false;
    }
    return i2c_initialized[port];
}

esp_err_t i2c_manager_deinit(i2c_port_t port) {
    if (!i2c_initialized[port]) {
        return ESP_OK; // Already deinitialized
    }

    esp_err_t ret = i2c_driver_delete(port);
    if (ret == ESP_OK) {
        i2c_initialized[port] = false;
    }
    return ret;
} 