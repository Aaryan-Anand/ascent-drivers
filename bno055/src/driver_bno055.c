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

bno055_powermode_t bno_getpowermode(void) {
    bno_setpage(0);
    bno055_powermode_t powermode = bnoreadRegister(BNO_PWR_MODE_ADDR) & 0x03;
    return powermode;
}

void bno_setoprmode(bno055_opmode_t mode) {
    // Attempt to set the mode and verify it
    for (int i = 0; i < 3; i++) {
        bno_setpage(0);
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
    // Attempt to set the mode and verify it
    for (int i = 0; i < 3; i++) {
        bno_setpage(0);
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

    uint8_t *data = bnoreadMultiple(BNO_GRV_DATA_X_MSB_ADDR, 6);
    if (data != NULL) {
    // Parse gravity acceleration data
    if (grav_x) *grav_x = (int16_t)((data[1] << 8) | data[0]);
    if (grav_y) *grav_y = (int16_t)((data[3] << 8) | data[2]);
    if (grav_z) *grav_z = (int16_t)((data[5] << 8) | data[4]);

    free(data);
    }
}

void bno_getselftest(bool *st_mcu, bool *st_gyr, bool *st_mag, bool *st_acc) {
    bno_setpage(0);

    uint8_t st_results = bnoreadRegister(BNO_ST_RESULT_ADDR);

    if (st_mcu) *st_mcu = (st_results >> 0) & 0x01;
    if (st_gyr) *st_gyr = (st_results >> 1) & 0x01;
    if (st_mag) *st_mag = (st_results >> 2) & 0x01;
    if (st_acc) *st_acc = (st_results >> 3) & 0x01;
}

void bno_getinterruptstatus(bool *acc_nm, bool *acc_am, bool *acc_high_g, bool *gyr_drdy, bool *gyr_high_ratem, bool *gyro_am, bool *mag_drdy, bool *acc_bsx_drdy) {
    bno_setpage(0);

    uint8_t interrupts = bnoreadRegister(BNO_INT_STA_ADDR);

    if(acc_nm) *acc_nm = (interrupts >> 7) & 0x01;
    if(acc_am) *acc_am = (interrupts >> 6) & 0x01;
    if(acc_high_g) *acc_high_g = (interrupts >> 5) & 0x01;
    if(gyr_drdy) *gyr_drdy = (interrupts >> 4) & 0x01;
    if(gyr_high_ratem) *gyr_high_ratem = (interrupts >> 3) & 0x01;
    if(gyro_am) *gyro_am = (interrupts >> 2) & 0x01;
    if(mag_drdy) *mag_drdy = (interrupts >> 1) & 0x01;
    if(acc_bsx_drdy) *acc_bsx_drdy = (interrupts >> 0) & 0x01; 
}

void bno_getinterruptmask(bool *acc_nm, bool *acc_am, bool *acc_high_g, bool *gyr_drdy, bool *gyr_high_ratem, bool *gyro_am, bool *mag_drdy, bool *acc_bsx_drdy) {
    bno_setpage(1);
    
    uint8_t mask = bnoreadRegister(0x0F);

    if(acc_nm) *acc_nm = (mask >> 7) & 0x01;
    if(acc_am) *acc_am = (mask >> 6) & 0x01;
    if(acc_high_g) *acc_high_g = (mask >> 5) & 0x01;
    if(gyr_drdy) *gyr_drdy = (mask >> 4) & 0x01;
    if(gyr_high_ratem) *gyr_high_ratem = (mask >> 3) & 0x01;
    if(gyro_am) *gyro_am = (mask >> 2) & 0x01;
    if(mag_drdy) *mag_drdy = (mask >> 1) & 0x01;
    if(acc_bsx_drdy) *acc_bsx_drdy = (mask >> 0) & 0x01; 
}

void bno_setinterruptmask(bool acc_nm, bool acc_am, bool acc_high_g, bool gyr_drdy, bool gyr_high_ratem, bool gyro_am, bool mag_drdy, bool acc_bsx_drdy) {
    bno_setpage(1);
    
    uint8_t mask = 0;
    if(acc_nm) mask |= (1 << 7);
    if(acc_am) mask |= (1 << 6);
    if(acc_high_g) mask |= (1 << 5);
    if(gyr_drdy) mask |= (1 << 4);
    if(gyr_high_ratem) mask |= (1 << 3);
    if(gyro_am) mask |= (1 << 2);
    if(mag_drdy) mask |= (1 << 1);
    if(acc_bsx_drdy) mask |= (1 << 0);

    bnowriteRegister(INT_MSK_ADDR, mask);
}

void bno_getinterruptenable(bool *acc_nm, bool *acc_am, bool *acc_high_g, bool *gyr_drdy, bool *gyr_high_ratem, bool *gyro_am, bool *mag_drdy, bool *acc_bsx_drdy) {
    bno_setpage(1);

    uint8_t enabled = bnoreadRegister(INT_ADDR);

    if(acc_nm) *acc_nm = (enabled >> 7) & 0x01;
    if(acc_am) *acc_am = (enabled >> 6) & 0x01;
    if(acc_high_g) *acc_high_g = (enabled >> 5) & 0x01;
    if(gyr_drdy) *gyr_drdy = (enabled >> 4) & 0x01;
    if(gyr_high_ratem) *gyr_high_ratem = (enabled >> 3) & 0x01;
    if(gyro_am) *gyro_am = (enabled >> 2) & 0x01;
    if(mag_drdy) *mag_drdy = (enabled >> 1) & 0x01;
    if(acc_bsx_drdy) *acc_bsx_drdy = (enabled >> 0) & 0x01; 
}

void bno_setinterruptenable(bool acc_nm, bool acc_am, bool acc_high_g, bool gyr_drdy, bool gyr_high_ratem, bool gyro_am, bool mag_drdy, bool acc_bsx_drdy) {
    bno_setpage(1);
    
    uint8_t enabled = 0;
    if(acc_nm) enabled |= (1 << 7);
    if(acc_am) enabled |= (1 << 6);
    if(acc_high_g) enabled |= (1 << 5);
    if(gyr_drdy) enabled |= (1 << 4);
    if(gyr_high_ratem) enabled |= (1 << 3);
    if(gyro_am) enabled |= (1 << 2);
    if(mag_drdy) enabled |= (1 << 1);
    if(acc_bsx_drdy) enabled |= (1 << 0);

    bnowriteRegister(INT_ADDR, enabled);
}

bool bno_getclockstatus(void){
    bno_setpage(0);

    uint8_t sys_clk_status = bnoreadRegister(0x38);

    return ((sys_clk_status >> 1) & 0x01);
}

uint8_t bno_getsysstatus(void){
    bno_setpage(0);

    return bnoreadRegister(BNO_SYS_STATUS_ADDR);
}

uint8_t bno_getsyserror(void){
    bno_setpage(0);

    return bnoreadRegister(BNO_SYS_ERR_ADDR);
}

void bno_get_units(bool *ORI_android_windows, bool *temp_unit, bool *eul_unit, bool *gyr_unit, bool *acc_unit) {
    bno_setpage(0);

    uint8_t units = bnoreadRegister(BNO_UNIT_SEL_ADDR);

    if (ORI_android_windows) *ORI_android_windows = (units & 0x80) != 0;
    if (temp_unit) *temp_unit = (units & 0x10) != 0;
    if (eul_unit) *eul_unit = (units & 0x08) != 0;
    if (gyr_unit) *gyr_unit = (units & 0x04) != 0;
    if (acc_unit) *acc_unit = (units & 0x02) != 0;
}

uint8_t bno_gettemp(void) {
    bno_setpage(0);

    return bnoreadRegister(BNO_TEMP_ADDR);
}

void bno_trigger_st(void) {
    bno_setpage(0);

    bnowriteRegister(BNO_SYS_TRIGGER_ADDR,0x01);
}

void bno_trigger_rst(void) {
    bno_setpage(0);

    bnowriteRegister(BNO_SYS_TRIGGER_ADDR,0x20);
}

void bno_trigger_int_rst(void) {
    bno_setpage(0);
    
    bnowriteRegister(BNO_SYS_TRIGGER_ADDR,0x40);
}



bno055_tempsource_t bno_get_tempsource(void) {
    bno_setpage(0);

    return bnoreadRegister(0x40) & 0x01;
}

void bno_set_tempsource(bno055_tempsource_t source) {
    bno_setpage(0);
    
    // Attempt to set the temperature source and verify it
    for (int i = 0; i < 3; i++) {
        bnowriteRegister(0x40, source);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        
        bno055_tempsource_t current_source = bno_get_tempsource();
        if (current_source == source) {
            return; // Exit if the temperature source is set correctly
        }
    }

    ESP_LOGE(TAG, "Failed to set temp source to %d after 3 attempts", source);
}

bno_axismap bno_get_axismapconfig(void) {
    bno_setpage(0);

    bno_axismap axis_map;
    uint8_t axis_map_reg = bnoreadRegister(0x41);

    axis_map.x = (axis_map_reg & 0x03);        // Bits 1-0: X-axis mapping
    axis_map.y = (axis_map_reg & 0x0C) >> 2;   // Bits 3-2: Y-axis mapping
    axis_map.z = (axis_map_reg & 0x30) >> 4;   // Bits 5-4: Z-axis mapping

    return axis_map;
}

void bno_set_axismapconfig(bno_axismap axis_map) {
    bno_setpage(0);

    // Ensure all values are valid (0=X, 1=Y, 2=Z) and no duplicates
    if (axis_map.x > 2 || axis_map.y > 2 || axis_map.z > 2 ||
        axis_map.x == axis_map.y || axis_map.x == axis_map.z || axis_map.y == axis_map.z) {
        ESP_LOGE(TAG, "Invalid axis mapping configuration!");
        return;
    }

    uint8_t axis_map_reg = 0x00;
    axis_map_reg |= (axis_map.x & 0x03);       
    axis_map_reg |= (axis_map.y & 0x03) << 2;  
    axis_map_reg |= (axis_map.z & 0x03) << 4;  

    // Write and verify
    bnowriteRegister(0x41, axis_map_reg);
    uint8_t verify = bnoreadRegister(0x41);
    if (verify != axis_map_reg) {
        ESP_LOGE(TAG, "Axis map configuration mismatch: expected 0x%02X, got 0x%02X", axis_map_reg, verify);
    }
}

void bno_get_axismapsign(bool *x, bool *y, bool *z) {
    bno_setpage(0);

    uint8_t signs = bnoreadRegister(0x42);

    if (x) *x = (signs & 0x01) != 0;
    if (y) *y = (signs & 0x02) != 0;
    if (z) *z = (signs & 0x04) != 0;
}

void bno_set_axismapsign(bool x, bool y, bool z) {
    bno_setpage(0);

    uint8_t signs = 0x00;
    if (x) signs |= 0x01;
    if (y) signs |= 0x02;
    if (z) signs |= 0x04;

    bnowriteRegister(0x42, signs);
    uint8_t verify = bnoreadRegister(0x42);
    if (verify != signs) {
        ESP_LOGE(TAG, "Axis sign configuration mismatch: expected 0x%02X, got 0x%02X", signs, verify);
    }
}


uint8_t bno_getpage(void) {
    return bnoreadRegister(0x07); // Read the page ID from register 0x7
}

static uint8_t current_page = 0; // Static variable to hold the current page

void bno_setpage(int8_t page) {
    if (current_page == page) {
        return; // Exit if the current page is already the desired page
    }
    bnowriteRegister(0x07, page);
    current_page = page; // Update the static variable with the new page
}