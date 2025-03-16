// initial driver that reads and writes registers, and initializes the sensor to read data

#include <stdio.h>
#include <string.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "driver_bno055.h"

static const char *TAG = "BNO055";

// Static variables to store pin configurations
static int sda_pin;
static int scl_pin;
static int i2c_port;
static uint32_t i2c_freq;

esp_err_t bno055_init(int sda, int scl, int port, uint32_t freq) {
    // Store pin configurations
    sda_pin = sda;
    scl_pin = scl;
    i2c_port = port;
    i2c_freq = freq;

    // Initialize I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .master.clk_speed = i2c_freq,
    };
    i2c_param_config(i2c_port, &conf);
    return i2c_driver_install(i2c_port, conf.mode, 0, 0, 0);
}

uint8_t readRegister(uint8_t reg_addr) {
    uint8_t data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    // First, write the register address we want to read from
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BNO055_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    
    // Then perform a repeated start and read the data
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BNO055_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, &data, 1, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    return data;
}

esp_err_t writeRegister(uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BNO055_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    return ret;
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