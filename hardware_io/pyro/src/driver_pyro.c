#include "driver_pyro.h"
#include "ascent_r2_hardware_definition.h"
#include "adc_oneshot.h"
#include "esp_log.h"

static const char *TAG = "PYRO";
static adc_oneshot_unit_handle_t adc1_handle;

// Array to map channel numbers to GPIO pins
static const gpio_num_t pyro_out_pins[] = {
    PYRO1_OUT, PYRO2_OUT, PYRO3_OUT, PYRO4_OUT
};

esp_err_t pyro_init(void) {
    // Configure output pins
    for (int i = 0; i < 4; i++) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << pyro_out_pins[i]),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        ESP_ERROR_CHECK(gpio_config(&io_conf));
        gpio_set_level(pyro_out_pins[i], 0);
    }

    // Initialize ADC
    adc_oneshot_unit_init_cfg_t adc_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_config, &adc1_handle));

    return ESP_OK;
}

bool pyro_continuity(pyro_channel_t channel) {
    if (channel < 1 || channel > 4) {
        return false;
    }

    // Get the corresponding ADC channel from the enum
    adc_channel_t adc_chan;
    switch (channel) {
        case PYRO_CHANNEL_1:
            adc_chan = ADC_CHANNEL_0;
            break;
        case PYRO_CHANNEL_2:
            adc_chan = ADC_CHANNEL_1;
            break;
        case PYRO_CHANNEL_3:
            adc_chan = ADC_CHANNEL_2;
            break;
        case PYRO_CHANNEL_4:
            adc_chan = ADC_CHANNEL_3;
            break;
        default:
            return false;
    }

    // Configure ADC channel for this reading
    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, adc_chan, &channel_config));

    // Read ADC
    int adc_reading;
    esp_err_t ret = adc_oneshot_read(adc1_handle, adc_chan, &adc_reading);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC read error on channel %d", channel);
        return false;
    }

    // Convert ADC reading to voltage (3.3V reference, 12-bit ADC = 4095 max value)
    double voltage = (double)adc_reading * (3.3 / 4095.0);
    
    // Return true if voltage is above 1V threshold
    return (voltage > 1.0);
}

double pyro_resistance(pyro_channel_t channel) {
    if (channel < 1 || channel > 4) {
        return -1.0;
    }

    // Get the corresponding ADC channel from the enum
    adc_channel_t adc_chan;
    switch (channel) {
        case PYRO_CHANNEL_1:
            adc_chan = ADC_CHANNEL_0;
            break;
        case PYRO_CHANNEL_2:
            adc_chan = ADC_CHANNEL_1;
            break;
        case PYRO_CHANNEL_3:
            adc_chan = ADC_CHANNEL_2;
            break;
        case PYRO_CHANNEL_4:
            adc_chan = ADC_CHANNEL_3;
            break;
        default:
            return -1.0;
    }

    // Configure ADC channel for this reading
    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, adc_chan, &channel_config));

    // Read ADC
    int adc_reading;
    esp_err_t ret = adc_oneshot_read(adc1_handle, adc_chan, &adc_reading);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC read error on channel %d", channel);
        return -1.0;
    }

    // Convert ADC reading to voltage (3.3V reference, 12-bit ADC = 4095 max value)
    double measured_voltage = (double)adc_reading * (3.3 / 4095.0);
    
    // Scale voltage based on voltage divider (multiply by 3.4)
    double actual_voltage = measured_voltage * 3.4;

    ESP_LOGI(TAG, "Channel %d - ADC: %d, Measured V: %.3f, Actual V: %.3f", 
             channel, adc_reading, measured_voltage, actual_voltage);

    return actual_voltage;
}

esp_err_t pyro_activate(pyro_channel_t channel) {
    if (channel < 1 || channel > 4) {
        return ESP_ERR_INVALID_ARG;
    }

    // Check continuity before activation
    if (!pyro_continuity(channel)) {
        ESP_LOGE(TAG, "No continuity detected on channel %d", channel);
        return ESP_ERR_INVALID_STATE;
    }

    // Activate the pyro channel
    gpio_set_level(pyro_out_pins[channel - 1], 1);
    
    // You might want to add a delay here depending on your requirements
    // vTaskDelay(pdMS_TO_TICKS(100)); // Example: 100ms pulse

    // Reset the output
    gpio_set_level(pyro_out_pins[channel - 1], 0);

    return ESP_OK;
}
