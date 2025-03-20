#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "driver_psu.h"

static esp_err_t psuValidate(void) {
    printf("\n=== Power Supply Status ===\n");
    fflush(stdout);
    esp_err_t ret = ESP_OK;

    // Read battery voltage
    double voltage = psu_read_battery_voltage();
    if (voltage < 0) {
        printf("Battery voltage reading failed\n");
        ret = ESP_FAIL;
    } else {
        printf("Battery voltage: %.3f V\n", voltage);
    }

    // Check power source
    const char* power_source = psu_get_power_source();
    printf("Power source: %s\n", power_source);

    fflush(stdout);
    return ret;
}

void PSUTest(void) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    printf("\n\n=== Power Supply Hardware Test ===\n");
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Initialize PSU
    printf("Initializing PSU...\n");
    fflush(stdout);
    
    esp_err_t ret = psu_init();
    if (ret != ESP_OK) {
        printf("Failed to initialize PSU (error: %d)\n", ret);
        fflush(stdout);
        return;
    }
    printf("PSU initialization successful\n\n");
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Continuous monitoring loop
    printf("Starting continuous monitoring (10 readings)...\n");
    for (int i = 0; i < 10; i++) {
        ret = psuValidate();
        if (ret != ESP_OK) {
            printf("PSU validation failed (error: %d)\n", ret);
            return;
        }
        printf("\n");
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second between readings
    }

    printf("PSU test completed\n");
}
