#ifndef TEMP_CONTROL__H__
#define TEMP_CONTROL__H__

#include "esp_err.h"

esp_err_t temp_control_init(void);
void      temp_control_task(void *pvParameter);

#endif // TEMP_CONTROL__H__
