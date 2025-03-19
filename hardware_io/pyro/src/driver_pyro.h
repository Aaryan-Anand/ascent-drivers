#ifndef PYRO_H
#define PYRO_H

#include <stdbool.h>
#include "driver/gpio.h"
#include "esp_err.h"

// Pyro channel enumeration
typedef enum {
    PYRO_CHANNEL_1 = 1,    // Apogee
    PYRO_CHANNEL_2 = 2,    // Mains
    PYRO_CHANNEL_3 = 3,
    PYRO_CHANNEL_4 = 4
} pyro_channel_t;

// Function prototypes
esp_err_t pyro_init(void);
bool pyro_continuity(pyro_channel_t channel);
float pyro_resistance(pyro_channel_t channel);
esp_err_t pyro_activate(pyro_channel_t channel);

#endif /* PYRO_H */
