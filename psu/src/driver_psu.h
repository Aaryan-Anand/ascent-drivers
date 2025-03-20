#ifndef DRIVER_PSU_H
#define DRIVER_PSU_H

#include "esp_err.h"

// Function prototypes
esp_err_t psu_init(void);
double psu_read_battery_voltage(void);
const char* psu_get_power_source(void);
void psu_deinit(void);

#endif /* DRIVER_PSU_H */
