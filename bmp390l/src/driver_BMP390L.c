#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver_BMP390L.h"

static const char *TAG = "BMP390";

// Static variables to store pin configurations
static int sda_pin;
static int scl_pin;
static int i2c_port;
static uint32_t i2c_freq;

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
#define BMP390_I2C_ADDR 0x76

esp_err_t bmp390_init(int sda, int scl, int port, uint32_t freq) {
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

// Add other BMP390L functions here...
