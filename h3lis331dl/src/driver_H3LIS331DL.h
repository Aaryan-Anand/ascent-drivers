#ifndef DRIVER_H3LIS331DL_H
#define DRIVER_H3LIS331DL_H

#include "esp_err.h"
#include "driver/i2c.h"
#include "i2c_manager.h"

// Register definitions
#define H3LIS331DL_CTRL_REG1          0x20
#define H3LIS331DL_CTRL_REG4          0x23
#define H3LIS331DL_OUT_X_L            0x28

/**
 * @brief Initialize the H3LIS331DL sensor
 * 
 * @param port The I2C port number
 * @return esp_err_t ESP_OK on success
 */
esp_err_t h3lis331dl_init(i2c_port_t port);

/**
 * @brief Read accelerometer data
 * 
 * @param x_accel Pointer to store X-axis acceleration
 * @param y_accel Pointer to store Y-axis acceleration
 * @param z_accel Pointer to store Z-axis acceleration
 * @return esp_err_t ESP_OK on success
 */
esp_err_t h3lis331dl_read_accel(float *x_accel, float *y_accel, float *z_accel);

#endif /* DRIVER_H3LIS331DL_H */ 