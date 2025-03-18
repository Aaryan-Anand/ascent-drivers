// initial driver that reads and writes registers, and initializes the sensor to read data

#include <stdio.h>
#include <string.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "driver_bno055.h"
#include "i2c_manager.h"

static const char *TAG = "BNO055";

static int i2c_port;

esp_err_t bno055_init(i2c_port_t port) {
    // Store only the port number
    i2c_port = port;
    
    // Check if I2C is already initialized
    if (!i2c_manager_is_initialized(port)) {
        ESP_LOGE(TAG, "I2C port %d not initialized", port);
        return ESP_ERR_INVALID_STATE;
    }

    // Add any BNO055-specific initialization here
    // (chip ID verification, sensor configuration, etc.)

    return ESP_OK;
}

uint8_t readRegister(uint8_t reg_addr) {
    uint8_t data;
    i2c_manager_read_register(i2c_port, BNO055_I2C_ADDR, reg_addr, &data, 1);
    return data;
}

esp_err_t writeRegister(uint8_t reg_addr, uint8_t data) {
    return i2c_manager_write_register(i2c_port, BNO055_I2C_ADDR, reg_addr, &data, 1);
}

uint8_t bno055_get_mode(void) {
    uint8_t currentmode = readRegister(BNO_OPR_MODE_ADDR);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    return currentmode;
}

void bno055_set_mode(bno055_opmode_t mode) {
    writeRegister(BNO_OPR_MODE_ADDR, CONFIG);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    writeRegister(BNO_OPR_MODE_ADDR, mode);
    vTaskDelay(500 / portTICK_PERIOD_MS);
}

void bno055_get_calib(uint8_t *sys, uint8_t *gyro, uint8_t *accel, uint8_t *mag) {
    uint8_t calData = readRegister(BNO_CALIB_STAT_ADDR);
    if (sys != NULL) {
        *sys = (calData >> 6) & 0x03;
    }
    if (gyro != NULL) {
        *gyro = (calData >> 4) & 0x03;
    }
    if (accel != NULL) {
        *accel = (calData >> 2) & 0x03;
    }
    if (mag != NULL) {
        *mag = calData & 0x03;
    }
}

void bno_init() {
    bno055_set_mode(NDOF);
    ESP_LOGI(TAG,"BNO Initialized!");
}