#include "peltier_driver.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_log.h"

#include "modbus_params.h" // Move management of get/set params to "modbus_master" code?

// TODO: Add peltier module failure dectection (rate of change of temperature vs state, detection of flipped high/low
// side, etc.)

static const char *LOG_TAG = "peltier_driver";

#define PELTIER_HIGH_SIDE_RELAY_OUTPUT (gpio_num_t)3 // XIAO ESP32S3 A2
#define PELTIER_LOW_SIDE_RELAY_OUTPUT  (gpio_num_t)2 // XIAO ESP32S3 A1

#define PELTIER_TOGGLING_DELAY_MS      10

#define PELTIER_MUTEX_TIMEOUT_MS       10U

static PeltierDriver_Command_t s_current_command = PELTIER_DRIVER_COMMAND_NONE;
static PeltierDriver_Command_t s_requested_command = PELTIER_DRIVER_COMMAND_NONE;

static SemaphoreHandle_t s_current_command_mutex = NULL;
static SemaphoreHandle_t s_requested_command_mutex = NULL;

static inline bool command_invalid(PeltierDriver_Command_t command)
{
    return command != PELTIER_DRIVER_COMMAND_NONE && command != PELTIER_DRIVER_COMMAND_COOL
           && command != PELTIER_DRIVER_COMMAND_HEAT;
}

static esp_err_t set_current_command(PeltierDriver_Command_t command)
{
    if (command_invalid(command))
    {
        ESP_LOGE(LOG_TAG, "Invalid command!");
        return ESP_FAIL;
    }
    if (s_current_command_mutex == NULL)
    {
        ESP_LOGE(LOG_TAG, "Driver not initialized!");
        return ESP_FAIL;
    }

    if (xSemaphoreTake(s_current_command_mutex, pdMS_TO_TICKS(PELTIER_MUTEX_TIMEOUT_MS)) == pdFALSE)
    {
        ESP_LOGW(LOG_TAG, "Failed to take current command mutex!");
        return ESP_FAIL;
    }
    s_current_command = command;
    xSemaphoreGive(s_current_command_mutex);
    return ESP_OK;
}

static esp_err_t get_requested_command(PeltierDriver_Command_t *const command)
{
    if (command == NULL)
    {
        ESP_LOGE(LOG_TAG, "Command pointer null!");
        return ESP_FAIL;
    }
    if (s_requested_command_mutex == NULL)
    {
        ESP_LOGE(LOG_TAG, "Driver not initialized!");
        return ESP_FAIL;
    }

    if (xSemaphoreTake(s_requested_command_mutex, pdMS_TO_TICKS(PELTIER_MUTEX_TIMEOUT_MS)) == pdFALSE)
    {
        ESP_LOGW(LOG_TAG, "Failed to take requested command mutex!");
        return ESP_FAIL;
    }
    *command = s_requested_command;
    xSemaphoreGive(s_requested_command_mutex);
    return ESP_OK;
}

static esp_err_t enact_command(PeltierDriver_Command_t command)
{
    if (command_invalid(command))
    {
        ESP_LOGE(LOG_TAG, "Invalid command!");
        return ESP_FAIL;
    }
    PeltierDriver_Command_t current_command = command;
    esp_err_t               ret = peltier_driver_get_current_command(&current_command);
    if (command == current_command)
    {
        return ESP_OK;
    }

    esp_err_t high_ret = ESP_OK;
    esp_err_t low_ret = ESP_OK;

    if (command == PELTIER_DRIVER_COMMAND_NONE)
    {
        ESP_LOGI(LOG_TAG, "Disabling Peltier!");
        high_ret = gpio_set_level(PELTIER_HIGH_SIDE_RELAY_OUTPUT, 0);
        low_ret = gpio_set_level(PELTIER_LOW_SIDE_RELAY_OUTPUT, 0);
    }
    else if (command == PELTIER_DRIVER_COMMAND_COOL)
    {
        ESP_LOGI(LOG_TAG, "Setting Peltier to Cooling!");
        low_ret = gpio_set_level(PELTIER_LOW_SIDE_RELAY_OUTPUT, 0);
        if (low_ret == ESP_OK)
        {
            high_ret = gpio_set_level(PELTIER_HIGH_SIDE_RELAY_OUTPUT, 1);
        }
    }
    else if (command == PELTIER_DRIVER_COMMAND_HEAT)
    {
        ESP_LOGI(LOG_TAG, "Setting Peltier to Heating!");
        high_ret = gpio_set_level(PELTIER_HIGH_SIDE_RELAY_OUTPUT, 0);
        if (high_ret == ESP_OK)
        {
            low_ret = gpio_set_level(PELTIER_LOW_SIDE_RELAY_OUTPUT, 1);
        }
    }
    if (high_ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Failed to set gpio for PELTIER_HIGH_SIDE_RELAY_OUTPUT!");
        return high_ret;
    }
    if (low_ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Failed to set gpio for PELTIER_HIGH_SIDE_RELAY_OUTPUT!");
        return low_ret;
    }
    else
    {
        ret = set_current_command(command);
        if (ret != ESP_OK)
        {
            ESP_LOGE(LOG_TAG, "Failed to updated current command!");
        }
        // Refresh modbus registers
        ret = modbus_params_set_input_register_uint(PELTIER_DRIVER_STATE, (uint16_t)command);
        if (ret != ESP_OK)
        {
            ESP_LOGE(LOG_TAG, "Failed to update modbus parameter PELTIER_DRIVER_STATE!");
        }

        vTaskDelay(pdMS_TO_TICKS(PELTIER_TOGGLING_DELAY_MS));

        // TODO: Readback GPIO to confirm setting?

        return ret;
    }
}

esp_err_t peltier_driver_init(void)
{
    esp_err_t ret = ESP_OK;

    ret = gpio_sleep_set_pull_mode(PELTIER_HIGH_SIDE_RELAY_OUTPUT, GPIO_PULLUP_PULLDOWN);
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Failed to set sleep pull for gpio for PELTIER_HIGH_SIDE_RELAY_OUTPUT!");
        return ret;
    }

    ret = gpio_sleep_set_pull_mode(PELTIER_LOW_SIDE_RELAY_OUTPUT, GPIO_PULLUP_PULLDOWN);
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Failed to set sleep pull for gpio for PELTIER_LOW_SIDE_RELAY_OUTPUT!");
        return ret;
    }

    ret = gpio_set_pull_mode(PELTIER_HIGH_SIDE_RELAY_OUTPUT, GPIO_PULLUP_PULLDOWN);
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Failed to set pull for gpio for PELTIER_HIGH_SIDE_RELAY_OUTPUT!");
        return ret;
    }

    ret = gpio_set_pull_mode(PELTIER_LOW_SIDE_RELAY_OUTPUT, GPIO_PULLUP_PULLDOWN);
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Failed to set pull for gpio for PELTIER_LOW_SIDE_RELAY_OUTPUT!");
        return ret;
    }

    ret = gpio_set_direction(PELTIER_HIGH_SIDE_RELAY_OUTPUT, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Failed to set direction gpio for PELTIER_HIGH_SIDE_RELAY_OUTPUT!");
        return ret;
    }

    ret = gpio_set_direction(PELTIER_LOW_SIDE_RELAY_OUTPUT, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Failed to set direction gpio for PELTIER_LOW_SIDE_RELAY_OUTPUT!");
        return ret;
    }

    ret = gpio_set_level(PELTIER_HIGH_SIDE_RELAY_OUTPUT, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Failed to set gpio for PELTIER_HIGH_SIDE_RELAY_OUTPUT!");
        return ret;
    }

    ret = gpio_set_level(PELTIER_LOW_SIDE_RELAY_OUTPUT, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Failed to set gpio for PELTIER_LOW_SIDE_RELAY_OUTPUT!");
        return ret;
    }

    s_current_command = PELTIER_DRIVER_COMMAND_NONE;
    s_requested_command = PELTIER_DRIVER_COMMAND_NONE;

    s_current_command_mutex = xSemaphoreCreateMutex();
    if (s_current_command_mutex == NULL)
    {
        ESP_LOGE(LOG_TAG, "Failed to create current command mutex!");
        return ESP_FAIL;
    }

    s_requested_command_mutex = xSemaphoreCreateMutex();
    if (s_requested_command_mutex == NULL)
    {
        ESP_LOGE(LOG_TAG, "Failed to create requested command mutex!");
        return ESP_FAIL;
    }

    return ret;
}

void peltier_driver_task(void *pvParameter)
{
    while (1)
    {
        // Check if peltier override is requested by modbus master
        static bool last_peltier_override_state = false;
        bool        peltier_override_requested = false;
        esp_err_t   ret_override = modbus_params_get_coil_state(ENABLE_PELTIER_OVERRIDE, &peltier_override_requested);
        if (ret_override != ESP_OK)
        {
            ESP_LOGE(LOG_TAG, "Failed to get modbus parameter ENABLE_PELTIER_OVERRIDE!");
        }
        static bool last_cooling_override_state = false;
        bool        cooling_override_state = false;
        esp_err_t   ret_high = modbus_params_get_coil_state(OVERRIDE_PELTIER_COOLING, &cooling_override_state);
        if (ret_high != ESP_OK)
        {
            ESP_LOGE(LOG_TAG, "Failed to get modbus parameter OVERRIDE_PELTIER_COOLING!");
        }
        static bool last_heating_override_state = false;
        bool        heating_override_state = false;
        esp_err_t   ret_low = modbus_params_get_coil_state(OVERRIDE_PELTIER_HEATING, &heating_override_state);
        if (ret_low != ESP_OK)
        {
            ESP_LOGE(LOG_TAG, "Failed to get modbus parameter OVERRIDE_PELTIER_HEATING!");
        }

        if (ret_override == ESP_OK && ret_high == ESP_OK && ret_low == ESP_OK && peltier_override_requested)
        {
            if (!last_peltier_override_state)
            {
                ESP_LOGI(LOG_TAG, "Peltier override requested!");
                last_peltier_override_state = true;
            }
            if (last_cooling_override_state != cooling_override_state
                || last_heating_override_state != heating_override_state)
            {
                last_cooling_override_state = cooling_override_state;
                last_heating_override_state = heating_override_state;
                ESP_LOGI(LOG_TAG, "Peltier override state change!");
                if (cooling_override_state && !heating_override_state)
                {
                    ESP_LOGI(LOG_TAG, "Cooling override!");
                    ret_override = enact_command(PELTIER_DRIVER_COMMAND_COOL);
                }
                else if (!cooling_override_state && heating_override_state)
                {
                    ESP_LOGI(LOG_TAG, "Heating override!");
                    ret_override = enact_command(PELTIER_DRIVER_COMMAND_HEAT);
                }
                else
                {
                    ESP_LOGI(LOG_TAG, "Override to off!");
                    ret_override = enact_command(PELTIER_DRIVER_COMMAND_NONE);
                }
                if (ret_override != ESP_OK)
                {
                    ESP_LOGE(LOG_TAG, "Failed to enact override command!");
                }
            }
        }
        else
        {
            if (last_peltier_override_state)
            {
                ESP_LOGI(LOG_TAG, "Peltier override removed!");
                last_peltier_override_state = false;
            }

            PeltierDriver_Command_t requested_command = PELTIER_DRIVER_COMMAND_NONE;
            esp_err_t               ret = get_requested_command(&requested_command);
            if (ret == ESP_OK)
            {
                ret = enact_command(requested_command);
                if (ret != ESP_OK)
                {
                    ESP_LOGE(LOG_TAG, "Failed to enact requested command!");
                }
            }
            else
            {
                ESP_LOGE(LOG_TAG, "Failed to get requested command!");
            }
            // TODO: Add internal logic here!
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

esp_err_t peltier_driver_set_requested_command(PeltierDriver_Command_t command)
{
    if (command_invalid(command))
    {
        ESP_LOGE(LOG_TAG, "Invalid command!");
        return ESP_FAIL;
    }
    if (s_requested_command_mutex == NULL)
    {
        ESP_LOGE(LOG_TAG, "Driver not initialized!");
        return ESP_FAIL;
    }

    if (xSemaphoreTake(s_requested_command_mutex, pdMS_TO_TICKS(PELTIER_MUTEX_TIMEOUT_MS)) == pdFALSE)
    {
        ESP_LOGW(LOG_TAG, "Failed to take requested command mutex!");
        return ESP_FAIL;
    }
    if (command != s_requested_command)
    {
        s_requested_command = command;
    }
    xSemaphoreGive(s_requested_command_mutex);
    return ESP_OK;
}

esp_err_t peltier_driver_get_current_command(PeltierDriver_Command_t *const command)
{
    if (command == NULL)
    {
        ESP_LOGE(LOG_TAG, "Command pointer null!");
        return ESP_FAIL;
    }
    if (s_current_command_mutex == NULL)
    {
        ESP_LOGE(LOG_TAG, "Driver not initialized!");
        return ESP_FAIL;
    }

    if (xSemaphoreTake(s_current_command_mutex, pdMS_TO_TICKS(PELTIER_MUTEX_TIMEOUT_MS)) == pdFALSE)
    {
        ESP_LOGW(LOG_TAG, "Failed to take current command mutex!");
        return ESP_FAIL;
    }
    *command = s_current_command;
    xSemaphoreGive(s_current_command_mutex);
    return ESP_OK;
}
