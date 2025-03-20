#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "driver_bno055.h"
#include "ascent_r2_hardware_definition.h"
#include "i2c_manager.h"

void BNOTest() {
    printf("\n=== BNO055 Hardware Test ===\n");
    fflush(stdout);

    // Initialize BNO055 with pins from hardware definition
    /*esp_err_t ret = bno055_init(I2C_MASTER_SDA_IO, 
                               I2C_MASTER_SCL_IO, 
                               I2C_MASTER_PORT, 
                               I2C_MASTER_FREQ_HZ);
    if (ret != ESP_OK) {
        printf("Failed to initialize BNO055\n");
        return;
    }*/

    // Set BNO055 mode
    bno055_set_mode(NDOF);

    // Main loop
    printf("Reading BNO055 Fusion data...\n");
    for(int i = 0; i < 10; i++) {
        // Read heading
        uint8_t eul_h_lsb = bnoreadRegister(BNO_EUL_HEADING_LSB_ADDR);
        uint8_t eul_h_msb = bnoreadRegister(BNO_EUL_HEADING_MSB_ADDR);

        // Read pitch
        uint8_t eul_p_lsb = bnoreadRegister(BNO_EUL_PITCH_LSB_ADDR);
        uint8_t eul_p_msb = bnoreadRegister(BNO_EUL_PITCH_MSB_ADDR);

        // Read roll
        uint8_t eul_r_lsb = bnoreadRegister(BNO_EUL_ROLL_LSB_ADDR);
        uint8_t eul_r_msb = bnoreadRegister(BNO_EUL_ROLL_MSB_ADDR);

        // Combine MSB and LSB into a single value
        int16_t heading = (eul_h_msb << 8) | eul_h_lsb;
        int16_t pitch = (eul_p_msb << 8) | eul_p_lsb;
        int16_t roll = (eul_r_msb << 8) | eul_r_lsb;

        printf("Heading: %d, Pitch: %d, Roll: %d\n", heading, pitch, roll);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
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
}

void spit_out_data() {
    fflush(stdout);
    bno055_set_mode(NDOF);

    while(1) {
    int16_t acc_x, acc_y, acc_z, mag_x, mag_y, mag_z, gyr_x, gyr_y, gyr_z;
    bno_readamg(&acc_x, &acc_y, &acc_z, &mag_x, &mag_y, &mag_z, &gyr_x, &gyr_y, &gyr_z);

    printf("%d,%d,%d,%d,%d,%d,%d,%d,%d\n", acc_x, acc_y, acc_z, mag_x, mag_y, mag_z, gyr_x, gyr_y, gyr_z);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}