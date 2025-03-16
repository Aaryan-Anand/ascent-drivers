#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_SCL_IO           6      // SCL pin
#define I2C_MASTER_SDA_IO           5      // SDA pin
#define I2C_MASTER_NUM              0       // I2C master port number
#define I2C_MASTER_FREQ_HZ          400000  // I2C master clock frequency
#define H3LIS331DL_ADDR             0x18    // I2C address of H3LIS331DL (SA0 pin low)

#define CTRL_REG1                   0x20
#define CTRL_REG4                   0x23
#define OUT_X_L                     0x28

static const char *TAG = "H3LIS331DL";

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static esp_err_t h3lis331dl_write_reg(uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_write_to_device(I2C_MASTER_NUM, H3LIS331DL_ADDR, write_buf, sizeof(write_buf), pdMS_TO_TICKS(100));
}

static esp_err_t h3lis331dl_read_reg(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, H3LIS331DL_ADDR, &reg_addr, 1, data, len, pdMS_TO_TICKS(100));
}

void h3lis331dl_task(void *pvParameters)
{
    // Initialize H3LIS331DL
    esp_err_t ret = h3lis331dl_write_reg(CTRL_REG1, 0x37);  // Enable all axes, 50 Hz data rate
    if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to write accelerometer data");
    }
    
    ret = h3lis331dl_write_reg(CTRL_REG4, 0x00);  // ±100g full scale
    if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to write accelerometer data");
    }
    

    int16_t accel_data[3];
    uint8_t data[6];

    while (1) {
        // Read accelerometer data
        esp_err_t ret = h3lis331dl_read_reg(OUT_X_L | 0x80, data, 6);  // 0x80 for auto-increment
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
    ESP_ERROR_CHECK(i2c_master_init());
    xTaskCreate(h3lis331dl_task, "h3lis331dl_task", 2048, NULL, 5, NULL);
}
