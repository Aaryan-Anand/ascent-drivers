#ifndef DRIVER_BMP390L_H
#define DRIVER_BMP390L_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// Type definitions
typedef struct {
    uint64_t pressure;     // 24-bit raw pressure (stored in a 64-bit type)
    int64_t  temperature;  // 24-bit raw temperature
} bmp3_uncomp_data_t;

typedef struct {
    double pressure;       // Compensated pressure in Pa (floating point)
    double temperature;    // Compensated temperature in °C
} bmp3_data_t;

typedef struct {
    uint16_t par_t1;
    int16_t  par_t2;
    int8_t   par_t3;
    uint16_t par_p1;
    int16_t  par_p2;
    int8_t   par_p3;
    int8_t   par_p4;
    uint16_t par_p5;
    uint16_t par_p6;
    int8_t   par_p7;
    int8_t   par_p8;
    int16_t  par_p9;
    int16_t  par_p10;
    int8_t   par_p11;
    double   t_lin;
} bmp390_calib_data_t;

// Enums for sensor configuration
typedef enum {
    BMP390_MODE_SLEEP = 0x00,
    BMP390_MODE_FORCED = 0x01,
    BMP390_MODE_NORMAL = 0x03
} bmp390_mode_t;

typedef enum {
    BMP390_OVERSAMPLING_1X = 0,
    BMP390_OVERSAMPLING_2X = 1,
    BMP390_OVERSAMPLING_4X = 2,
    BMP390_OVERSAMPLING_8X = 3,
    BMP390_OVERSAMPLING_16X = 4,
    BMP390_OVERSAMPLING_32X = 5
} bmp390_osr_t;

typedef enum {
    BMP390_ODR_200HZ = 0,
    BMP390_ODR_100HZ = 1,
    BMP390_ODR_50HZ  = 2,
    BMP390_ODR_25HZ  = 3,
    BMP390_ODR_12_5HZ = 4,
    BMP390_ODR_6_25HZ = 5,
    BMP390_ODR_3_125HZ = 6,
    BMP390_ODR_1_5625HZ = 7,
    BMP390_ODR_0_78125HZ = 8
} bmp390_odr_t;

typedef enum {
    BMP390_IIR_FILTER_BYPASS = 0,
    BMP390_IIR_FILTER_COEFF_1 = 1,
    BMP390_IIR_FILTER_COEFF_3 = 2,
    BMP390_IIR_FILTER_COEFF_7 = 3,
    BMP390_IIR_FILTER_COEFF_15 = 4,
    BMP390_IIR_FILTER_COEFF_31 = 5,
    BMP390_IIR_FILTER_COEFF_63 = 6,
    BMP390_IIR_FILTER_COEFF_127 = 7
} bmp390_iir_filter_t;

// Configuration structures
typedef struct {
    bool press_en;
    bool temp_en;
    bmp390_mode_t mode;
} bmp390_pwr_ctrl_t;

typedef struct {
    bmp390_osr_t press_os;
    bmp390_osr_t temp_os;
} bmp390_osr_settings_t;

typedef struct {
    bmp390_iir_filter_t iir_filter;
} bmp390_config_t;

typedef struct {
    bool cmd_rdy;
} bmp390_status_t;

// Constants
#define BMP3_OK           0
#define BMP3_PRESS_TEMP   3
#define BMP3_PRESS        1
#define BMP3_TEMP         2

// Public function prototypes
esp_err_t bmp390_init(int sda, int scl, int port, uint32_t freq);
esp_err_t bmp390_read_sensor_data(double *pressure, double *temperature);
esp_err_t bmp390_read_calib_data(bmp390_calib_data_t *calib);

// Configuration function prototypes
esp_err_t bmp390_get_chip_id(uint8_t *chip_id);
esp_err_t bmp390_get_pwr_ctrl(bmp390_pwr_ctrl_t *ctrl);
esp_err_t bmp390_set_pwr_ctrl(const bmp390_pwr_ctrl_t *ctrl);
esp_err_t bmp390_get_osr(bmp390_osr_settings_t *osr);
esp_err_t bmp390_set_osr(const bmp390_osr_settings_t *osr);
esp_err_t bmp390_get_odr(bmp390_odr_t *odr);
esp_err_t bmp390_set_odr(bmp390_odr_t odr);
esp_err_t bmp390_get_config(bmp390_config_t *config);
esp_err_t bmp390_set_config(const bmp390_config_t *config);
esp_err_t bmp390_get_err_reg(uint8_t *err);
esp_err_t bmp390_get_status(bmp390_status_t *status);

#endif /* DRIVER_BMP390L_H */ 