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

    bno_setpage(0);

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

uint8_t bno_getoprmode(void) {
    bno_setpage(0);
    bno055_opmode_t currentmode = bnoreadRegister(BNO_OPR_MODE_ADDR);
    return currentmode;
}

uint8_t bno_getpowermode(void) {
    bno_setpage(0);
    bno055_powermode_t powermode = bnoreadRegister(BNO_PWR_MODE_ADDR) & 0x03;
    return powermode;
}

void bno_setoprmode(bno055_opmode_t mode) {
    bno_setpage(0);
    bnowriteRegister(BNO_OPR_MODE_ADDR, CONFIG);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    // Attempt to set the mode and verify it
    for (int i = 0; i < 3; i++) {
        bnowriteRegister(BNO_OPR_MODE_ADDR, mode);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        
        uint8_t current_mode = bno_getoprmode();
        if (current_mode == mode) {
            return; // Exit if the mode is set correctly
        }
    }
    
    ESP_LOGE(TAG, "Failed to set mode to %d after 3 attempts", mode);
}

void bno_setpowermode(bno055_powermode_t mode) {
    bno_setpage(0);
    bnowriteRegister (BNO_PWR_MODE_ADDR, mode);

    // Attempt to set the mode and verify it
    for (int i = 0; i < 3; i++) {
        bnowriteRegister(BNO_PWR_MODE_ADDR, mode);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        
        uint8_t current_mode = bno_getpowermode();
        if (current_mode == mode) {
            return; // Exit if the mode is set correctly
        }
    }

    ESP_LOGE(TAG, "Failed to set power mode to %d after 3 attempts", mode);
}

void bno_get_calib(uint8_t *sys, uint8_t *gyro, uint8_t *accel, uint8_t *mag) {
    bno_setpage(0);
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
    bno_setpage(0);
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

void bno_readeuler(int16_t *eul_heading,
                 int16_t *eul_roll,
                 int16_t *eul_pitch) {
    bno_setpage(0);
    uint8_t *data = bnoreadMultiple(0x1A, 6); // Read 18 registers from 0x08 to 0x19
    if (data != NULL) {
        // Parse heading
        if (eul_heading) *eul_heading = (int16_t)((data[1] << 8) | data[0]);

        // Parse roll
        if (eul_roll) *eul_roll = (int16_t)((data[3] << 8) | data[2]);

        // Parse pitch
        if (eul_pitch) *eul_pitch = (int16_t)((data[5] << 8) | data[4]);

        free(data); // Free the allocated memory
    }
}

void bno_readquart(int16_t *quart_w, int16_t *quart_x, int16_t *quart_y, int16_t *quart_z) {
    bno_setpage(0);

    uint8_t *data = bnoreadMultiple(0x20, 8);
    if (data != NULL) {
        if (quart_w) *quart_w = (int16_t)((data[1] << 8) | data[0]);
        if (quart_x) *quart_x = (int16_t)((data[3] << 8) | data[2]);
        if (quart_y) *quart_y = (int16_t)((data[5] << 8) | data[4]);
        if (quart_z) *quart_z = (int16_t)((data[7] << 8) | data[6]);

        free(data);
    }
}

void bno_readlia(int16_t *lia_x, int16_t *lia_y, int16_t *lia_z) {
    bno_setpage(0);

    uint8_t *data = bnoreadMultiple(0x28,6);
    if (data != NULL) {
        // Parse linear acceleration data
        if (lia_x) *lia_x = (int16_t)((data[1] << 8) | data[0]);
        if (lia_y) *lia_y = (int16_t)((data[3] << 8) | data[2]);
        if (lia_z) *lia_z = (int16_t)((data[5] << 8) | data[4]);

        free(data); // Free the allocated memory
    }
}

void bno_readgrav(int16_t *grav_x, int16_t *grav_y, int16_t *grav_z) {
    bno_setpage(0);

    uint8_t *data = bnoreadMultiple(0x2F, 6);
    if (data != NULL) {
    // Parse gravity acceleration data
    if (grav_x) *grav_x = (int16_t)((data[1] << 8) | data[0]);
    if (grav_y) *grav_y = (int16_t)((data[3] << 8) | data[2]);
    if (grav_z) *grav_z = (int16_t)((data[5] << 8) | data[4]);

    free(data);
    }
}

void bno_getselftest(bool *st_mcu, bool *st_gyr, bool *st_mag, bool *st_acc) {
    uint8_t st_results = bnoreadRegister(0x36);

    if (st_mcu) *st_mcu = (st_results >> 0) & 0x01;
    if (st_gyr) *st_gyr = (st_results >> 1) & 0x01;
    if (st_mag) *st_mag = (st_results >> 2) & 0x01;
    if (st_acc) *st_acc = (st_results >> 3) & 0x01;
}

void bno_getinterrupts(bool *acc_nm, bool *acc_am, bool *acc_high_g, bool *gyr_drdy, bool *gyr_high_ratem, bool *mag_drdy, bool *acc_bsx_drdy) {
    uint8_t interrupts = bnoreadRegister(0x37);

    if(acc_nm) *acc_nm  = (interrupts >> 7) & 0x01;
    if(acc_am) *acc_am  = (interrupts >> 6) & 0x01;
    if(acc_high_g) *acc_high_g  = (interrupts >> 5) & 0x01;
    if(gyr_drdy) *gyr_drdy  = (interrupts >> 4) & 0x01;
    if(gyr_high_ratem) *gyr_high_ratem  = (interrupts >> 3) & 0x01;
    if(mag_drdy) *mag_drdy  = (interrupts >> 2) & 0x01;
    if(acc_bsx_drdy) *acc_bsx_drdy  = (interrupts >> 1) & 0x01;
}

bool bno_getclockstatus(void){
    uint8_t sys_clk_status = bnoreadRegister(0x38);

    return ((sys_clk_status >> 1) & 0x01);
}

uint8_t bno_getsysstatus(void){
    return bnoreadRegister(0x39);
}

uint8_t bno_getsyserror(void){
    return bnoreadRegister(0x3A);
}

void bno_get_units(bool *ORI_android_windows, bool *temp_unit, bool *eul_unit, bool *gyr_unit, bool *acc_unit) {
    uint8_t units = bnoreadRegister(0x3B);

    if (ORI_android_windows) *ORI_android_windows = (units & 0x80) != 0;
    if (temp_unit) *temp_unit = (units & 0x10) != 0;
    if (eul_unit) *eul_unit = (units & 0x08) != 0;
    if (gyr_unit) *gyr_unit = (units & 0x04) != 0;
    if (acc_unit) *acc_unit = (units & 0x02) != 0;
}

uint8_t bno_gettemp(void) {
    return bnoreadRegister(0x34);
}

void bno_trigger_st(void) {
    bnowriteRegister(0x3F,0x01);
}

void bno_trigger_rst(void) {
    bnowriteRegister(0x3F,0x20);
}

void bno_trigger_int_rst(void) {
    bnowriteRegister(0x3F,0x40);
}

uint8_t bno_getpage(void) {
    return bnoreadRegister(0x07); // Read the page ID from register 0x7
}

void bno_setpage(int8_t page) {
    bnowriteRegister(0x07, page);
}