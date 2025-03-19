#include "pyro.h"
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

    return ESP_OK;
}

bool pyro_continuity(pyro_channel_t channel) {
    if (channel < 1 || channel > 4) {
        return false;
    }
    
    return gpio_get_level(pyro_cont_pins[channel - 1]);
}

float pyro_resistance(pyro_channel_t channel) {
    if (channel < 1 || channel > 4) {
        return -1.0f;
    }

    // Note: This is a placeholder implementation
    // You'll need to implement the actual ADC reading and resistance calculation
    // based on your hardware configuration
    uint32_t adc_reading = 0; // Replace with actual ADC reading
    float voltage = adc_reading * (3.3f / 4095.0f); // Assuming 3.3V reference and 12-bit ADC
    float current = 0.001f; // Replace with your actual current source value
    float resistance = voltage / current;

    return resistance;
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
