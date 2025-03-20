#include "driver_psu.h"
#include "ascent_r2_hardware_definition.h"
#include "adc_oneshot.h"
#include "esp_log.h"
#include "adc_cali.h"
#include "adc_cali_scheme.h"

static const char *TAG = "PSU";
static adc_oneshot_unit_handle_t adc2_handle;
static adc_cali_handle_t adc_cali_handle;

esp_err_t psu_init(void) {
    // Initialize ADC2
    adc_oneshot_unit_init_cfg_t adc_config = {
        .unit_id = ADC_UNIT_2,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_config, &adc2_handle));

    // Set up ADC calibration
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_2,
        .chan = ADC_CHANNEL_6,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle));

    return ESP_OK;
}

double psu_read_battery_voltage(void) {
    // Configure ADC channel for battery reading (GPIO17 = ADC2_CHANNEL_6)
    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, ADC_CHANNEL_6, &channel_config));

    // Read ADC
    int adc_raw;
    int voltage_mv;
    esp_err_t ret = adc_oneshot_read(adc2_handle, ADC_CHANNEL_6, &adc_raw);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC read error on battery voltage pin");
        return -1.0;
    }

    // Convert raw reading to millivolts using calibration
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, adc_raw, &voltage_mv));
    
    // Convert from mV to V and apply voltage divider scaling
    double measured_voltage = voltage_mv / 1000.0;
    double actual_voltage = measured_voltage * 3.778;

    ESP_LOGI(TAG, "Battery ADC Raw: %d, Calibrated: %dmV, Actual: %.3fV", 
             adc_raw, voltage_mv, actual_voltage);

    return actual_voltage;
}

const char* psu_get_power_source(void) {
    double actual_voltage = psu_read_battery_voltage();
    
    // If actual voltage reading is less than 7.556V (2V * 3.778) or invalid (-1.0), assume USB power
    if (actual_voltage < 7.556) {
        ESP_LOGI(TAG, "Power source: USB");
        return "USB";
    } else {
        ESP_LOGI(TAG, "Power source: Battery");
        return "Battery";
    }
}

void psu_deinit(void) {
    adc_cali_delete_scheme_curve_fitting(adc_cali_handle);
    adc_oneshot_del_unit(adc2_handle);
}
