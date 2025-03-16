#ifndef DRIVER_BMP390L_H
#define DRIVER_BMP390L_H

#include "esp_err.h"

/**
 * @brief Initialize the BMP390L sensor
 * 
 * @param sda The GPIO pin number for SDA
 * @param scl The GPIO pin number for SCL
 * @param port The I2C port number
 * @param freq The I2C frequency in Hz
 * @return esp_err_t ESP_OK on success
 */
esp_err_t bmp390_init(int sda, int scl, int port, uint32_t freq);

/**
 * @brief Read pressure and temperature data from the sensor
 * 
 * @param pressure Pointer to store pressure value in hPa
 * @param temperature Pointer to store temperature value in °C
 * @return esp_err_t ESP_OK on success
 */
esp_err_t bmp390_read_sensor_data(float *pressure, float *temperature);

#endif /* DRIVER_BMP390L_H */ 