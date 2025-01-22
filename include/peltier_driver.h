#ifndef PELTIER_DRIVER__H__
#define PELTIER_DRIVER__H__

#include "esp_err.h"
#include <stdint.h>

typedef enum
{
    PELTIER_DRIVER_COMMAND_NONE = 0,
    PELTIER_DRIVER_COMMAND_COOL,
    PELTIER_DRIVER_COMMAND_HEAT,
} PeltierDriver_Command_t;

esp_err_t peltier_driver_init(void);
void      peltier_driver_task(void *pvParameter);

esp_err_t peltier_driver_set_requested_command(PeltierDriver_Command_t command);
esp_err_t peltier_driver_get_current_command(PeltierDriver_Command_t *const command);

#endif // PELTIER_DRIVER__H__
