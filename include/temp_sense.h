#ifndef TEMP_SENSE__H__
#define TEMP_SENSE__H__

#include "esp_err.h"

esp_err_t temp_sense_init(void);
void temp_sense_task(void *pvParameter);

esp_err_t temp_sense_get_last_measurement_temperature(float *const temperature_degc);

#endif // TEMP_SENSE__H__