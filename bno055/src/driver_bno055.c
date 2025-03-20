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

uint8_t bnoreadRegister(uint8_t reg_addr) {
    uint8_t data;
    i2c_manager_read_register(i2c_port, BNO055_I2C_ADDR, reg_addr, &data, 1);
    return data;
}

uint8_t* bnoreadMultiple(uint8_t reg_addr, size_t length) {
    uint8_t *data = malloc(length); // Allocate memory for the data
    if (data != NULL) {
        i2c_manager_read_register(i2c_port, BNO055_I2C_ADDR, reg_addr, data, length);
    }
    return data; // Return the pointer to the data
}

esp_err_t bnowriteRegister(uint8_t reg_addr, uint8_t data) {
    return i2c_manager_write_register(i2c_port, BNO055_I2C_ADDR, reg_addr, &data, 1);
}

uint8_t bno055_get_mode(void) {
    uint8_t currentmode = bnoreadRegister(BNO_OPR_MODE_ADDR);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    return currentmode;
}

void bno055_set_mode(bno055_opmode_t mode) {
    bnowriteRegister(BNO_OPR_MODE_ADDR, CONFIG);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    // Attempt to set the mode and verify it
    for (int i = 0; i < 3; i++) {
        bnowriteRegister(BNO_OPR_MODE_ADDR, mode);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        
        uint8_t current_mode = bno055_get_mode();
        if (current_mode == mode) {
            ESP_LOGI(TAG, "Mode set successfully to %d", mode);
            return; // Exit if the mode is set correctly
        }
    }
    
    ESP_LOGE(TAG, "Failed to set mode to %d after 3 attempts", mode);
}

void bno055_get_calib(uint8_t *sys, uint8_t *gyro, uint8_t *accel, uint8_t *mag) {
    uint8_t calData = bnoreadRegister(BNO_CALIB_STAT_ADDR);
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

void bno_readamg(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z,
                 int16_t *mag_x, int16_t *mag_y, int16_t *mag_z,
                 int16_t *gyr_x, int16_t *gyr_y, int16_t *gyr_z) {
    uint8_t *data = bnoreadMultiple(0x08, 18); // Read 18 registers from 0x08 to 0x19
    if (data != NULL) {
        // Parse accelerometer data
        if (acc_x) *acc_x = (int16_t)((data[1] << 8) | data[0]);
        if (acc_y) *acc_y = (int16_t)((data[3] << 8) | data[2]);
        if (acc_z) *acc_z = (int16_t)((data[5] << 8) | data[4]);

        // Parse magnetometer data
        if (mag_x) *mag_x = (int16_t)((data[7] << 8) | data[6]);
        if (mag_y) *mag_y = (int16_t)((data[9] << 8) | data[8]);
        if (mag_z) *mag_z = (int16_t)((data[11] << 8) | data[10]);

        // Parse gyroscope data
        if (gyr_x) *gyr_x = (int16_t)((data[13] << 8) | data[12]);
        if (gyr_y) *gyr_y = (int16_t)((data[15] << 8) | data[14]);
        if (gyr_z) *gyr_z = (int16_t)((data[17] << 8) | data[16]);

        free(data); // Free the allocated memory
    }
}


void bno_init() {
    bno055_set_mode(NDOF);
    ESP_LOGI(TAG,"BNO Initialized!");
}