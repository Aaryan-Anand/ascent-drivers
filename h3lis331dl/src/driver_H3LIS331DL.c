#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver_H3LIS331DL.h"
#include "i2c_manager.h"

static const char *TAG = "H3LIS331DL";

#define H3LIS331DL_I2C_ADDR 0x18    // H3LIS331DL I2C address

// Modify init function to just store port number
static i2c_port_t current_i2c_port;

esp_err_t h3lis331dl_init(i2c_port_t port) {
    // Store only the port number since I2C is initialized in main
    current_i2c_port = port;
    
    // Check if I2C is already initialized
    if (!i2c_manager_is_initialized(port)) {
        ESP_LOGE(TAG, "I2C port %d not initialized", port);
        return ESP_ERR_INVALID_STATE;
    }
    
    return ESP_OK;
}

// Update register write function to use i2c_manager
static esp_err_t h3lis331dl_write_reg(uint8_t reg_addr, uint8_t data)
{
    return i2c_manager_write_register(current_i2c_port, H3LIS331DL_I2C_ADDR, 
                                    reg_addr, &data, 1);
}

// Update register read function to use i2c_manager
static esp_err_t h3lis331dl_read_reg(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_manager_read_register(current_i2c_port, H3LIS331DL_I2C_ADDR,
                                   reg_addr, data, len);
}

void h3lis331dl_task(void *pvParameters)
{
    // Initialize H3LIS331DL
    esp_err_t ret = h3lis331dl_write_reg(H3LIS331DL_CTRL_REG1, 0x37);  // Enable all axes, 50 Hz data rate
    if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to write accelerometer data");
    }
    
    ret = h3lis331dl_write_reg(H3LIS331DL_CTRL_REG4, 0x00);  // ±100g full scale
    if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to write accelerometer data");
    }
    

    int16_t accel_data[3];
    uint8_t data[6];

    while (1) {
        // Read accelerometer data
        esp_err_t ret = h3lis331dl_read_reg(H3LIS331DL_OUT_X_L | 0x80, data, 6);  // 0x80 for auto-increment
        if (ret == ESP_OK) {
            accel_data[0] = (int16_t)((data[1] << 8) | data[0]);  // X-axis
            accel_data[1] = (int16_t)((data[3] << 8) | data[2]);  // Y-axis
            accel_data[2] = (int16_t)((data[5] << 8) | data[4]);  // Z-axis

            ESP_LOGI(TAG, "Accel: X=%d, Y=%d, Z=%d", accel_data[0], accel_data[1], accel_data[2]);
        } else {
            ESP_LOGE(TAG, "Failed to read accelerometer data");
        }

        vTaskDelay(pdMS_TO_TICKS(100));  // Delay for 100ms
    }
}

void app_main(void)
{
    // Only create the task since I2C is initialized in main.c
    xTaskCreate(h3lis331dl_task, "h3lis331dl_task", 2048, NULL, 5, NULL);
}
