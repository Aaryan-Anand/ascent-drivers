#include "driver_pyro.h"
#include "ascent_r2_hardware_definition.h"
#include "driver/adc.h"
#include "esp_log.h"

static const char *TAG = "PYRO";

// Arrays to map channel numbers to GPIO pins
static const gpio_num_t pyro_out_pins[] = {
    PYRO1_OUT, PYRO2_OUT, PYRO3_OUT, PYRO4_OUT
};

static const gpio_num_t pyro_cont_pins[] = {
    PYRO1_CONT, PYRO2_CONT, PYRO3_CONT, PYRO4_CONT
};

// Update the ADC channel mappings to match the actual GPIO pins
static const adc1_channel_t pyro_adc_channels[] = {
    ADC1_CHANNEL_0,  // For PYRO1_CONT (GPIO 1)
    ADC1_CHANNEL_1,  // For PYRO2_CONT (GPIO 2)
    ADC1_CHANNEL_2,  // For PYRO3_CONT (GPIO 3)
    ADC1_CHANNEL_3   // For PYRO4_CONT (GPIO 4)
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

    // Configure continuity pins
    for (int i = 0; i < 4; i++) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << pyro_cont_pins[i]),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        ESP_ERROR_CHECK(gpio_config(&io_conf));
    }

    // Configure ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    
    // Configure ADC channels
    for (int i = 0; i < 4; i++) {
        adc1_config_channel_atten(pyro_adc_channels[i], ADC_ATTEN_DB_11);
    }

    return ESP_OK;
}

bool pyro_continuity(pyro_channel_t channel) {
    if (channel < 1 || channel > 4) {
        return false;
    }
    
    return gpio_get_level(pyro_cont_pins[channel - 1]);
}

double pyro_resistance(pyro_channel_t channel) {
    if (channel < 1 || channel > 4) {
        return -1.0;
    }

    // Read ADC value
    int adc_reading = adc1_get_raw(pyro_adc_channels[channel - 1]);
    if (adc_reading < 0) {
        ESP_LOGE(TAG, "ADC read error on channel %d", channel);
        return -1.0;
    }

    // Convert ADC reading to voltage (3.3V reference, 12-bit ADC = 4095 max value)
    double measured_voltage = (double)adc_reading * (3.3 / 4095.0);
    
    // Scale voltage based on voltage divider (multiply by 3.4)
    double actual_voltage = measured_voltage * 3.4;

    ESP_LOGI(TAG, "Channel %d - ADC: %d, Measured V: %.3f, Actual V: %.3f", 
             channel, adc_reading, measured_voltage, actual_voltage);

    // For now, just return the actual voltage
    // We'll implement resistance calculation in the next step
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
