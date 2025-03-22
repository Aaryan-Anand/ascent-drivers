#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "driver_bno055.h"
#include "ascent_r2_hardware_definition.h"
#include "i2c_manager.h"

void BNOTest() {
    printf("\n=== BNO055 Full Test ===\n");
    fflush(stdout);
    
    // Initialize the BNO055 sensor
    esp_err_t ret = bno055_init(I2C_NUM_0);
    if (ret != ESP_OK) {
        printf("Failed to initialize BNO055: %d\n", ret);
        return;
    }

    // Set operation mode to NDOF
    bno_setoprmode(NDOF);
    printf("Operation mode set to NDOF.\n");

    // Read and print calibration status
    uint8_t sys, gyro, accel, mag;
    bno_get_calib(&sys, &gyro, &accel, &mag);
    printf("Calibration Status - Sys: %d, Gyro: %d, Accel: %d, Mag: %d\n", sys, gyro, accel, mag);

    // Read and print system status
    uint8_t sys_status = bno_getsysstatus();
    printf("System Status: %d\n", sys_status);

    // Read and print system error
    uint8_t sys_error = bno_getsyserror();
    printf("System Error: %d\n", sys_error);

    // Read and print clock status
    bool clock_status = bno_getclockstatus();
    printf("Clock Status: %s\n", clock_status ? "Configured State" : "Free to Configure");

    // Read and print units
    bool ori_unit, temp_unit, eul_unit, gyr_unit, acc_unit;
    bno_get_units(&ori_unit, &temp_unit, &eul_unit, &gyr_unit, &acc_unit);
    printf("Units - Orientation: %s, Temperature: %s, Euler: %s, Gyroscope: %s, Acceleration: %s\n",
       ori_unit ? "Android" : "Windows", 
       temp_unit ? "Fahrenheit" : "Celcius",
       eul_unit ? "Radians" : "Degrees",
       gyr_unit ? "Rps" : "Dps",
       acc_unit ? "mg" : "m/s^2");

    // Read and print temperature
    uint8_t temperature = bno_gettemp();
    printf("Temperature: %d °C\n", temperature);

    // Read and print interrupt status
    bool acc_nm, acc_am, acc_high_g, gyr_drdy, gyr_high_ratem, mag_drdy, acc_bsx_drdy;
    bno_getinterrupts(&acc_nm, &acc_am, &acc_high_g, &gyr_drdy, &gyr_high_ratem, &mag_drdy, &acc_bsx_drdy);
    printf("Interrupts - Acc NM: %s, Acc AM: %s, Acc High G: %s, Gyro DRDY: %s, Gyro High Rate: %s, Mag DRDY: %s, Acc BSX DRDY: %s\n",
           acc_nm ? "Yes" : "No", acc_am ? "Yes" : "No", acc_high_g ? "Yes" : "No",
           gyr_drdy ? "Yes" : "No", gyr_high_ratem ? "Yes" : "No", mag_drdy ? "Yes" : "No", acc_bsx_drdy ? "Yes" : "No");

    // Read and print self-test results
    bool st_mcu, st_gyr, st_mag, st_acc;
    bno_getselftest(&st_mcu, &st_gyr, &st_mag, &st_acc);
    printf("Self-Test Results - MCU: %s, Gyro: %s, Mag: %s, Acc: %s\n",
           st_mcu ? "Passed" : "Failed", st_gyr ? "Passed" : "Failed",
           st_mag ? "Passed" : "Failed", st_acc ? "Passed" : "Failed");

    bno_setoprmode(AMG);
    printf("\n");

    printf("Reading BNO055 AMG data...\n");
    for(int i = 0; i < 10; i++) {
        int16_t acc_x, acc_y, acc_z, mag_x, mag_y, mag_z, gyr_x, gyr_y, gyr_z;
        bno_readamg(&acc_x, &acc_y, &acc_z, &mag_x, &mag_y, &mag_z, &gyr_x, &gyr_y, &gyr_z);

        printf("Accel X: %d, Accel Y: %d, Accel Z: %d\n", acc_x, acc_y, acc_z);
        printf("Mag X: %d, Mag Y: %d, Mag Z: %d\n", mag_x, mag_y, mag_z);
        printf("Gyro X: %d, Gyro Y: %d, Gyro Z: %d\n", gyr_x, gyr_y, gyr_z);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    printf("\n");

    bno_setoprmode(NDOF);

    printf("Reading BNO055 EUL data...\n");
    for(int i = 0; i < 10; i++) {
        int16_t eul_heading, eul_roll, eul_pitch;
        bno_readeuler(&eul_heading, &eul_roll, &eul_pitch);

        printf("Heading: %d, Roll: %d, Pitch: %d\n", eul_heading, eul_roll, eul_pitch);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    printf("\n");

    printf("BNO055 Full Test Completed.\n");
}

void spit_out_data() {
    fflush(stdout);
    bno_setoprmode(NDOF);

    while(1) {
    int16_t acc_x, acc_y, acc_z, mag_x, mag_y, mag_z, gyr_x, gyr_y, gyr_z;
    bno_readamg(&acc_x, &acc_y, &acc_z, &mag_x, &mag_y, &mag_z, &gyr_x, &gyr_y, &gyr_z);

    printf("%d,%d,%d,%d,%d,%d,%d,%d,%d\n", acc_x, acc_y, acc_z, mag_x, mag_y, mag_z, gyr_x, gyr_y, gyr_z);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}