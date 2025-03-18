#ifndef DRIVER_H3LIS331DL_H
#define DRIVER_H3LIS331DL_H

#include "esp_err.h"
#include "driver/i2c.h"
#include "i2c_manager.h"

// Register definitions
#define H3LIS331DL_WHO_AM_I         0x0F
#define H3LIS331DL_CTRL_REG1          0x20
#define H3LIS331DL_CTRL_REG2          0x21
#define H3LIS331DL_CTRL_REG3          0x22
#define H3LIS331DL_CTRL_REG4          0x23
#define H3LIS331DL_OUT_X_L            0x28
#define H3LIS331DL_CTRL_REG5          0x24
#define H3LIS331DL_HP_FILTER_RESET    0x25
#define H3LIS331DL_REFERENCE          0x26

// Axes enable configuration (bits 0-2)
typedef enum {
    H3LIS331DL_AXIS_NONE = 0x00,
    H3LIS331DL_AXIS_X = 0x01,
    H3LIS331DL_AXIS_Y = 0x02,
    H3LIS331DL_AXIS_Z = 0x04,
    H3LIS331DL_AXIS_XY = 0x03,
    H3LIS331DL_AXIS_XZ = 0x05,
    H3LIS331DL_AXIS_YZ = 0x06,
    H3LIS331DL_AXIS_XYZ = 0x07
} h3lis331dl_axes_config_t;

// Data rate configuration (bits 3-4)
typedef enum {
    H3LIS331DL_DATARATE_50HZ = 0x00,    // (00)
    H3LIS331DL_DATARATE_100HZ = 0x08,   // (01)
    H3LIS331DL_DATARATE_400HZ = 0x10,   // (10)
    H3LIS331DL_DATARATE_1000HZ = 0x18   // (11)
} h3lis331dl_datarate_t;

// Power mode configuration (bits 5-7)
typedef enum {
    H3LIS331DL_POWER_DOWN = 0x00,       // (000)
    H3LIS331DL_NORMAL = 0x20,           // (001)
    H3LIS331DL_LOW_POWER_0_5HZ = 0x40,  // (010)
    H3LIS331DL_LOW_POWER_1HZ = 0x60,    // (011)
    H3LIS331DL_LOW_POWER_2HZ = 0x80,    // (100)
    H3LIS331DL_LOW_POWER_5HZ = 0xA0,    // (101)
    H3LIS331DL_LOW_POWER_10HZ = 0xC0    // (110)
} h3lis331dl_power_mode_t;

// High-pass filter cutoff frequency configuration (bits 0-1)
typedef enum {
    H3LIS331DL_HPCF_NORMAL_MODE_00 = 0x00,    // Normal mode (00)
    H3LIS331DL_HPCF_NORMAL_MODE_10 = 0x02,    // Normal mode (10)
    H3LIS331DL_HPCF_REF_MODE = 0x01           // Reference signal for filtering (01)
} h3lis331dl_hpcf_t;

// High-pass filter enable for interrupts (bits 2-3)
typedef enum {
    H3LIS331DL_HP_DISABLED = 0x00,
    H3LIS331DL_HP1_ENABLED = 0x04,
    H3LIS331DL_HP2_ENABLED = 0x08,
    H3LIS331DL_HP_BOTH_ENABLED = 0x0C
} h3lis331dl_hp_interrupt_t;

// Filter data selection (bit 4)
typedef enum {
    H3LIS331DL_FILTER_BYPASS = 0x00,
    H3LIS331DL_FILTER_ENABLE = 0x10
} h3lis331dl_filter_t;

// High-pass filter mode (bits 5-6)
typedef enum {
    H3LIS331DL_HPM_NORMAL_RESET = 0x00,      // Normal mode (reset by reading REFERENCE)
    H3LIS331DL_HPM_REFERENCE = 0x20,         // Reference signal for filtering
    H3LIS331DL_HPM_NORMAL = 0x40,            // Normal mode
    H3LIS331DL_HPM_AUTO_RESET = 0x60         // Auto-reset on interrupt event
} h3lis331dl_hpm_t;

// Data signal on INT1 control (bits 0-1)
typedef enum {
    H3LIS331DL_INT1_SRC = 0x00,         // INT1 source
    H3LIS331DL_INT1_OR_INT2_SRC = 0x01, // INT1 or INT2 source
    H3LIS331DL_INT1_DRDY = 0x02,        // Data ready
    H3LIS331DL_INT1_BOOT = 0x03         // Boot running
} h3lis331dl_int1_cfg_t;

// Data signal on INT2 control (bits 3-4)
typedef enum {
    H3LIS331DL_INT2_SRC = 0x00,         // INT2 source
    H3LIS331DL_INT2_OR_INT1_SRC = 0x08, // INT1 or INT2 source
    H3LIS331DL_INT2_DRDY = 0x10,        // Data ready
    H3LIS331DL_INT2_BOOT = 0x18         // Boot running
} h3lis331dl_int2_cfg_t;

// Interrupt active level configuration (bit 7)
typedef enum {
    H3LIS331DL_INT_ACTIVE_HIGH = 0x00,
    H3LIS331DL_INT_ACTIVE_LOW = 0x80
} h3lis331dl_int_level_t;

// Interrupt pin output mode (bit 6)
typedef enum {
    H3LIS331DL_INT_PUSH_PULL = 0x00,
    H3LIS331DL_INT_OPEN_DRAIN = 0x40
} h3lis331dl_int_pin_mode_t;

// Full scale selection (bits 4-5)
typedef enum {
    H3LIS331DL_SCALE_100G = 0x00,    // ±100g
    H3LIS331DL_SCALE_200G = 0x10,    // ±200g
    H3LIS331DL_SCALE_400G = 0x30     // ±400g (note: 11 binary)
} h3lis331dl_scale_t;

// SPI interface mode (bit 0)
typedef enum {
    H3LIS331DL_SPI_4_WIRE = 0x00,   // 4-wire SPI interface
    H3LIS331DL_SPI_3_WIRE = 0x01    // 3-wire SPI interface
} h3lis331dl_spi_mode_t;

// Data endianness (bit 6)
typedef enum {
    H3LIS331DL_BIG_ENDIAN = 0x00,    // MSB at lower address (default)
    H3LIS331DL_LITTLE_ENDIAN = 0x40  // LSB at lower address
} h3lis331dl_endian_t;

// Block data update (bit 7)
typedef enum {
    H3LIS331DL_BDU_CONTINUOUS = 0x00,  // Continuous update
    H3LIS331DL_BDU_BLOCKED = 0x80      // Output registers not updated until MSB and LSB read
} h3lis331dl_bdu_t;

// Sleep-to-Wake function (bits 0-1)
typedef enum {
    H3LIS331DL_SLEEP_TO_WAKE_DISABLED = 0x00,  // Sleep-to-wake disabled
    H3LIS331DL_SLEEP_TO_WAKE_LOW_POWER = 0x03  // Sleep-to-wake enabled in low power mode
} h3lis331dl_sleep_to_wake_t;

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

/**
 * @brief Read the chip ID from the WHO_AM_I register
 * 
 * @param[out] chip_id Pointer to store the chip ID value
 * @return esp_err_t ESP_OK on success
 */
esp_err_t h3lis331dl_get_chip_id(uint8_t *chip_id);

// Function declarations for reading and writing each setting
esp_err_t h3lis331dl_get_axes_config(h3lis331dl_axes_config_t *axes_config);
esp_err_t h3lis331dl_set_axes_config(h3lis331dl_axes_config_t axes_config);

esp_err_t h3lis331dl_get_datarate(h3lis331dl_datarate_t *datarate);
esp_err_t h3lis331dl_set_datarate(h3lis331dl_datarate_t datarate);

esp_err_t h3lis331dl_get_power_mode(h3lis331dl_power_mode_t *power_mode);
esp_err_t h3lis331dl_set_power_mode(h3lis331dl_power_mode_t power_mode);

// Function declarations for CTRL_REG2 settings
esp_err_t h3lis331dl_get_hpcf(h3lis331dl_hpcf_t *hpcf);
esp_err_t h3lis331dl_set_hpcf(h3lis331dl_hpcf_t hpcf);

esp_err_t h3lis331dl_get_hp_interrupt(h3lis331dl_hp_interrupt_t *hp_int);
esp_err_t h3lis331dl_set_hp_interrupt(h3lis331dl_hp_interrupt_t hp_int);

esp_err_t h3lis331dl_get_filter_mode(h3lis331dl_filter_t *filter);
esp_err_t h3lis331dl_set_filter_mode(h3lis331dl_filter_t filter);

esp_err_t h3lis331dl_get_hp_mode(h3lis331dl_hpm_t *hp_mode);
esp_err_t h3lis331dl_set_hp_mode(h3lis331dl_hpm_t hp_mode);

esp_err_t h3lis331dl_reboot_memory(void);

// Function declarations for CTRL_REG3 settings
esp_err_t h3lis331dl_get_int1_config(h3lis331dl_int1_cfg_t *int_cfg);
esp_err_t h3lis331dl_set_int1_config(h3lis331dl_int1_cfg_t int_cfg);

esp_err_t h3lis331dl_get_int2_config(h3lis331dl_int2_cfg_t *int_cfg);
esp_err_t h3lis331dl_set_int2_config(h3lis331dl_int2_cfg_t int_cfg);

esp_err_t h3lis331dl_get_int_level(h3lis331dl_int_level_t *level);
esp_err_t h3lis331dl_set_int_level(h3lis331dl_int_level_t level);

esp_err_t h3lis331dl_get_int_pin_mode(h3lis331dl_int_pin_mode_t *mode);
esp_err_t h3lis331dl_set_int_pin_mode(h3lis331dl_int_pin_mode_t mode);

esp_err_t h3lis331dl_get_int1_latch(bool *latch);
esp_err_t h3lis331dl_set_int1_latch(bool latch);

esp_err_t h3lis331dl_get_int2_latch(bool *latch);
esp_err_t h3lis331dl_set_int2_latch(bool latch);

// Function declarations for CTRL_REG4 settings
esp_err_t h3lis331dl_get_scale(h3lis331dl_scale_t *scale);
esp_err_t h3lis331dl_set_scale(h3lis331dl_scale_t scale);

esp_err_t h3lis331dl_get_spi_mode(h3lis331dl_spi_mode_t *mode);
esp_err_t h3lis331dl_set_spi_mode(h3lis331dl_spi_mode_t mode);

esp_err_t h3lis331dl_get_endian(h3lis331dl_endian_t *endian);
esp_err_t h3lis331dl_set_endian(h3lis331dl_endian_t endian);

esp_err_t h3lis331dl_get_bdu(h3lis331dl_bdu_t *bdu);
esp_err_t h3lis331dl_set_bdu(h3lis331dl_bdu_t bdu);

// Function declarations for CTRL_REG5 settings
esp_err_t h3lis331dl_get_sleep_to_wake(h3lis331dl_sleep_to_wake_t *mode);
esp_err_t h3lis331dl_set_sleep_to_wake(h3lis331dl_sleep_to_wake_t mode);

/**
 * @brief Reset the high-pass filter
 * 
 * Reading this register resets the high-pass filter. When the high-pass filter
 * is enabled and the REFERENCE register is used (HPM = 01), the filter output 
 * will be set to zero, and the content of the REFERENCE register is copied into
 * the filter registers. This allows the filter to overcome its settling time.
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t h3lis331dl_hp_filter_reset(void);

/**
 * @brief Set the reference value for high-pass filter
 * 
 * When the high-pass filter is enabled (HPen bits) and HPM bits are set to 01,
 * the filter output is generated taking this value as a reference.
 * 
 * @param reference Reference value for the high-pass filter
 * @return esp_err_t ESP_OK on success
 */
esp_err_t h3lis331dl_set_reference(uint8_t reference);

/**
 * @brief Get the current reference value for high-pass filter
 * 
 * @param[out] reference Pointer to store the reference value
 * @return esp_err_t ESP_OK on success
 */
esp_err_t h3lis331dl_get_reference(uint8_t *reference);

#endif /* DRIVER_H3LIS331DL_H */ 