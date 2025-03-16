#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_SCL_IO 6        // SCL pin
#define I2C_MASTER_SDA_IO 5        // SDA pin
#define I2C_MASTER_NUM I2C_NUM_0   // I2C port number
#define I2C_MASTER_FREQ_HZ 400000  // I2C master clock frequency
#define BMP390_ADDR 0x76           // BMP390 I2C address
#define BMP390_byte 0x01
#define BMP390_MAX_READ_LEN 6

static const char *TAG = "BMP390";

// BMP390 register addresses
#define BMP390_REG_CHIP_ID 0x00
#define BMP390_REG_PWR_CTRL 0x1B
#define BMP390_REG_OSR 0x1C
#define BMP390_REG_ODR 0x1D
#define BMP390_REG_CONFIG 0x1F
#define BMP390_REG_PRESS_DATA 0x04
#define BMP390_REG_TEMP_DATA 0x07
#define BMP390_REG_ERR_REG 0x02
#define BMP390_REG_STATUS 0x03

#define BMP390_STATUS_CMD_RDY 0x10

// Function prototypes
static esp_err_t i2c_master_init(void);
static esp_err_t bmp390_init(void);
static esp_err_t bmp390_read_sensor_data(float *pressure, float *temperature);
static esp_err_t bmp390_read_data_and_print(uint8_t addr, uint8_t len);
inline static esp_err_t cmd_error_check(void);

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(10000));
    ESP_LOGI(TAG, "BMP390 initializing.....");
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_ERROR_CHECK(bmp390_init());
    vTaskDelay(pdMS_TO_TICKS(1000));
    while (1) {
        float pressure, temperature;
        
        esp_err_t ret = bmp390_read_sensor_data(&pressure, &temperature);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Pressure: %.2f hPa, Temperature: %.2f °C", pressure, temperature);
        } else {
            ESP_LOGE(TAG, "Failed to read sensor data");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

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
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) return ret;
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

/*Start up sequence:
* 
* Check for chip ID
* Do a soft reset
* Wait until it is ready to take commands
* Configure oversampling settings
* Configure prescalar settings
* Configure IIR filter
* Configure power control ( normal mode enable pressure and temperature sensors)
* Wait until it is ready to take commands
* print ("Setup successful")
*
*/
static esp_err_t bmp390_init(void)
{
    uint8_t chip_id;
    esp_err_t ret;
    uint8_t status;

    // Read chip ID
    ret = i2c_master_write_read_device(I2C_MASTER_NUM, BMP390_ADDR, &(uint8_t){BMP390_REG_CHIP_ID}, 1, &chip_id, BMP390_byte, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) return ret;
    if (cmd_error_check() != ESP_OK) return ret;
    if (chip_id != 0x60) {
        ESP_LOGE(TAG, "Unexpected chip ID: 0x%02X", chip_id);
        return ESP_FAIL;
    }

    // Soft reset the sensor
    uint8_t soft_reset[2] = {0x7E, 0xB6};
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, BMP390_ADDR, soft_reset, sizeof(soft_reset), pdMS_TO_TICKS(100));
    if (ret != ESP_OK) return ret;
    if (cmd_error_check() != ESP_OK) return ret;
    ESP_LOGI(TAG, "soft reset.... wait a sec");
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Checks if it is ready to take any commands
    ret = i2c_master_write_read_device(I2C_MASTER_NUM, BMP390_ADDR, &(uint8_t){BMP390_REG_STATUS}, 1, &status, BMP390_byte, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) return ret;
    if (cmd_error_check() != ESP_OK) return ret;
    while ((status & BMP390_STATUS_CMD_RDY) == 0) {
        ESP_LOGI(TAG, "Waiting for CMD to be ready ....");
        ESP_LOGE(TAG, "status: 0x%02X", status);
        bmp390_read_data_and_print(BMP390_REG_STATUS,BMP390_byte);
        // Wait for cmd_rdy bit to be set
    }

    // Configure oversampling settings
    uint8_t osr_data[] = {BMP390_REG_OSR, 0x02};  // 2x oversampling for pressure and temperature
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, BMP390_ADDR, osr_data, sizeof(osr_data), pdMS_TO_TICKS(100));
    if (ret != ESP_OK) return ret;
    if (cmd_error_check() != ESP_OK) return ret;

    // Configure prescalar settings
    uint8_t odr_data[] = {BMP390_REG_ODR, 0x02};  // 2x prescalar for pressure and temperature
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, BMP390_ADDR, odr_data, sizeof(odr_data), pdMS_TO_TICKS(100));
    if (ret != ESP_OK) return ret;
    if (cmd_error_check() != ESP_OK) return ret;

    // Configure IIR filter
    uint8_t config_data[] = {BMP390_REG_CONFIG, 0x02};  // IIR filter coefficient: 3
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, BMP390_ADDR, config_data, sizeof(config_data), pdMS_TO_TICKS(100));
    if (ret != ESP_OK) return ret;
    if (cmd_error_check() != ESP_OK) return ret;

    // Configure power control ( normal mode enable pressure and temperature sensors)
    uint8_t pwr_ctrl_data[] = {BMP390_REG_PWR_CTRL, 0x33};
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, BMP390_ADDR, pwr_ctrl_data, sizeof(pwr_ctrl_data), pdMS_TO_TICKS(100));
    if (ret != ESP_OK) return ret;
    if (cmd_error_check() != ESP_OK) return ret;

    // Checks if it is ready to take any commands
    ret = i2c_master_write_read_device(I2C_MASTER_NUM, BMP390_ADDR, &(uint8_t){BMP390_REG_STATUS}, 1, &status, BMP390_byte, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) return ret;
    if (cmd_error_check() != ESP_OK) return ret;
    while ((status & BMP390_STATUS_CMD_RDY) == 0) {
        ESP_LOGI(TAG, "Waiting for CMD to be ready ....");
        ESP_LOGE(TAG, "status: 0x%02X", status);
        bmp390_read_data_and_print(BMP390_REG_STATUS,BMP390_byte);
        // Wait for cmd_rdy bit to be set
    }

    bmp390_read_data_and_print(BMP390_REG_CHIP_ID, BMP390_byte);
    bmp390_read_data_and_print(BMP390_REG_PWR_CTRL, BMP390_byte);
    bmp390_read_data_and_print(BMP390_REG_OSR, BMP390_byte);
    bmp390_read_data_and_print(BMP390_REG_ODR, BMP390_byte);
    bmp390_read_data_and_print(BMP390_REG_CONFIG, BMP390_byte);
    bmp390_read_data_and_print(BMP390_REG_STATUS, BMP390_byte);

    ESP_LOGI(TAG, "BMP390 initialized successfully");
    return ESP_OK;
}

inline static esp_err_t cmd_error_check()
{
    esp_err_t ret;
    uint8_t error = 0u;

    ret = i2c_master_write_read_device(I2C_MASTER_NUM, BMP390_ADDR, &(uint8_t){BMP390_REG_ERR_REG}, 1, &error, BMP390_byte, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "I2C write read failed!");
        return ESP_FAIL;
    }
    else if (error != 0x00) {
        ESP_LOGE(TAG, "Configuration error occured : 0x%02X", error);
        return ESP_FAIL;
    }
    else 
    {
        return ESP_OK;
    }
}

static esp_err_t bmp390_read_sensor_data(float *pressure, float *temperature)
{
    uint8_t data[6];
    esp_err_t ret;

    // Read pressure and temperature data
    ret = i2c_master_write_read_device(I2C_MASTER_NUM, BMP390_ADDR, &(uint8_t){BMP390_REG_PRESS_DATA}, 1, data, 6, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) return ret;

    // Convert raw data to pressure and temperature
    int32_t raw_pressure = (data[2] << 16) | (data[1] << 8) | data[0];
    int32_t raw_temperature = (data[5] << 16) | (data[4] << 8) | data[3];
    ESP_LOGI(TAG, "Pressure raw: %ld, Temperature raw: %ld", raw_pressure, raw_temperature);
    // TODO: Implement compensation algorithm as per the datasheet
    // For simplicity, we're using dummy conversion here
    *pressure = raw_pressure / 100.0f;
    *temperature = raw_temperature / 100.0f;

    return ESP_OK;
}

static esp_err_t bmp390_read_data_and_print(uint8_t addr, uint8_t len)
{
    uint8_t data[BMP390_MAX_READ_LEN];
    esp_err_t ret;
    ret = i2c_master_write_read_device(I2C_MASTER_NUM, BMP390_ADDR, &(uint8_t){addr}, 1, data, len, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) return ret;
    for(int i = 0; i < len; i++)
    {
        ESP_LOGI(TAG, "0x%02x byte %d: %02x", addr,i, data[i]);
    }

    return ESP_OK;
}
