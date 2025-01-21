#include "temp_sense.h"

#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "ntc_driver.h"

#include "modbus_params.h"

#define TEMP_SENSE_DEFAULT_PERIOD_MS 1000U

static const char *LOG_TAG = "temp_sense";

ntc_config_t ntc_config = {
    .b_value = 3435,
    .r25_ohm = 10000,
    .fixed_ohm = 10000,
    .vdd_mv = 3300,
    .circuit_mode = CIRCUIT_MODE_NTC_GND,
    .atten = ADC_ATTEN_DB_11,
    .channel = ADC_CHANNEL_0,
    .unit = ADC_UNIT_1,
};

ntc_device_handle_t ntc = NULL;
adc_oneshot_unit_handle_t adc_handle = NULL;

esp_err_t temp_sense_init(void)
{
    esp_err_t ret = ESP_OK;
    ret = ntc_dev_create(&ntc_config, &ntc, &adc_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "ntc_dev_create failed");
        return ret;
    }
    ret = ntc_dev_get_adc_handle(ntc, &adc_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "ntc_dev_get_adc_handle failed");
        return ret;
    }
    return ret;
}

void temp_sense_task(void *pvParameter)
{
    uint16_t task_period_ms = TEMP_SENSE_DEFAULT_PERIOD_MS;
    esp_err_t ret_esp = ESP_OK;

    ret_esp = modbus_params_set_holding_register_uint(TEMP_SENSE_PERIOD_MS, task_period_ms);
    if (ret_esp != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Failed to set modbus parameter TEMP_SENSE_PERIOD_MS!");
    }

    while (1)
    {
        // Get NTC sensor data
        float ntc_temperature_degc = NAN;
        esp_err_t ret = ntc_dev_get_temperature(ntc, &ntc_temperature_degc);
        if (ret == ESP_OK)
        {
            ESP_LOGV(LOG_TAG, "NTC temperature = %.2f°C.", ntc_temperature_degc);
        }
        else
        {
            ESP_LOGE(LOG_TAG, "Failed to get sensor data");
        }

        // Refresh modbus registers
        ret_esp = modbus_params_set_input_register_float(ACTUAL_TEMP_DEGC, ntc_temperature_degc);
        if (ret_esp != ESP_OK)
        {
            ESP_LOGE(LOG_TAG, "Failed to updated modbus parameter TEMP_TEMP_DEGC!");
        }

        // Update task period if requested
        uint16_t new_task_period_ms;
        ret_esp = modbus_params_get_holding_register_uint(TEMP_SENSE_PERIOD_MS, &new_task_period_ms);
        if (ret_esp != ESP_OK)
        {
            ESP_LOGE(LOG_TAG, "Failed to get modbus parameter TEMP_SENSE_PERIOD_MS!");
        }
        else if (new_task_period_ms != 0 && task_period_ms != new_task_period_ms)
        {
            ESP_LOGI(LOG_TAG, "Measurement period updated from %d to %dms!", task_period_ms, new_task_period_ms);
            task_period_ms = new_task_period_ms;
        }

        vTaskDelay(pdMS_TO_TICKS(task_period_ms));
    }
    ESP_ERROR_CHECK(ntc_dev_delete(ntc));
}
