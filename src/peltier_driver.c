#include "peltier_driver.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_log.h"

#include "modbus_params.h"

static const char *LOG_TAG = "peltier_driver";

#define PELTIER_HIGH_SIDE_RELAY_OUTPUT (gpio_num_t)6
#define PELTIER_LOW_SIDE_RELAY_OUTPUT (gpio_num_t)7

#define PELTIER_TOGGLING_DELAY_MS 10

esp_err_t peltier_driver_init(void)
{
    esp_err_t ret = ESP_OK;

    gpio_set_direction(PELTIER_HIGH_SIDE_RELAY_OUTPUT, GPIO_MODE_OUTPUT);
    gpio_set_direction(PELTIER_LOW_SIDE_RELAY_OUTPUT, GPIO_MODE_OUTPUT);

    gpio_set_level(PELTIER_HIGH_SIDE_RELAY_OUTPUT, 0);
    gpio_set_level(PELTIER_LOW_SIDE_RELAY_OUTPUT, 0);
    vTaskDelay(pdMS_TO_TICKS(PELTIER_TOGGLING_DELAY_MS));

    return ret;
}

void peltier_driver_task(void *pvParameter)
{
    while (1)
    {
        static bool last_peltier_override_state = false;
        // Check if peltier override is requested by modbus master
        bool peltier_override_requested = false;
        esp_err_t ret_override = get_coil_state(ENABLE_PELTIER_OVERRIDE, &peltier_override_requested);
        if (ret_override != ESP_OK)
        {
            ESP_LOGE(LOG_TAG, "Failed to get modbus parameter ENABLE_PELTIER_OVERRIDE!");
        }
        static bool last_high_side_override_state = false;
        bool high_side_override_state = false;
        esp_err_t ret_high = get_coil_state(OVERRIDE_PELTIER_HIGH_SIDE_RELAY_STATE, &high_side_override_state);
        if (ret_high != ESP_OK)
        {
            ESP_LOGE(LOG_TAG, "Failed to get modbus parameter OVERRIDE_PELTIER_HIGH_SIDE_RELAY_STATE!");
        }
        static bool last_low_side_override_state = false;
        bool low_side_override_state = false;
        esp_err_t ret_low = get_coil_state(OVERRIDE_PELTIER_LOW_SIDE_RELAY_STATE, &low_side_override_state);
        if (ret_low != ESP_OK)
        {
            ESP_LOGE(LOG_TAG, "Failed to get modbus parameter OVERRIDE_PELTIER_LOW_SIDE_RELAY_STATE!");
        }

        if (ret_override == ESP_OK && ret_high == ESP_OK && ret_low == ESP_OK && peltier_override_requested)
        {
            if (!last_peltier_override_state)
            {
                ESP_LOGI(LOG_TAG, "Peltier override requested!");
                last_peltier_override_state = true;
            }
            if (last_high_side_override_state != high_side_override_state || last_low_side_override_state != low_side_override_state)
            {
                last_high_side_override_state = high_side_override_state;
                last_low_side_override_state = low_side_override_state;
                ESP_LOGI(LOG_TAG, "Peltier override state change!");
                if (high_side_override_state && !low_side_override_state)
                {
                    ESP_LOGI(LOG_TAG, "High side override!");
                    gpio_set_level(PELTIER_LOW_SIDE_RELAY_OUTPUT, 0);
                    gpio_set_level(PELTIER_HIGH_SIDE_RELAY_OUTPUT, 1);
                    vTaskDelay(pdMS_TO_TICKS(PELTIER_TOGGLING_DELAY_MS));
                }
                else if (!high_side_override_state && low_side_override_state)
                {
                    ESP_LOGI(LOG_TAG, "Low side override!");
                    gpio_set_level(PELTIER_HIGH_SIDE_RELAY_OUTPUT, 0);
                    gpio_set_level(PELTIER_LOW_SIDE_RELAY_OUTPUT, 1);
                    vTaskDelay(pdMS_TO_TICKS(PELTIER_TOGGLING_DELAY_MS));
                }
                else
                {
                    ESP_LOGI(LOG_TAG, "Disable override!");
                    gpio_set_level(PELTIER_HIGH_SIDE_RELAY_OUTPUT, 0);
                    gpio_set_level(PELTIER_LOW_SIDE_RELAY_OUTPUT, 0);
                    vTaskDelay(pdMS_TO_TICKS(PELTIER_TOGGLING_DELAY_MS));
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
            gpio_set_level(PELTIER_HIGH_SIDE_RELAY_OUTPUT, 0);
            gpio_set_level(PELTIER_LOW_SIDE_RELAY_OUTPUT, 0);
            vTaskDelay(pdMS_TO_TICKS(PELTIER_TOGGLING_DELAY_MS));
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}