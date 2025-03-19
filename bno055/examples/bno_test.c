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
    printf("Reading BNO055 data...\n");
    for(int i = 0; i < 10; i++) {
        // Read heading
        uint8_t eul_h_lsb = readRegister(BNO_EUL_HEADING_LSB_ADDR);
        uint8_t eul_h_msb = readRegister(BNO_EUL_HEADING_MSB_ADDR);

        // Read pitch
        uint8_t eul_p_lsb = readRegister(BNO_EUL_PITCH_LSB_ADDR);
        uint8_t eul_p_msb = readRegister(BNO_EUL_PITCH_MSB_ADDR);

        // Read roll
        uint8_t eul_r_lsb = readRegister(BNO_EUL_ROLL_LSB_ADDR);
        uint8_t eul_r_msb = readRegister(BNO_EUL_ROLL_MSB_ADDR);

        // Combine MSB and LSB into a single value
        int16_t heading = (eul_h_msb << 8) | eul_h_lsb;
        int16_t pitch = (eul_p_msb << 8) | eul_p_lsb;
        int16_t roll = (eul_r_msb << 8) | eul_r_lsb;

        printf("Heading: %d, Pitch: %d, Roll: %d\n", heading, pitch, roll);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    printf("\n");
}