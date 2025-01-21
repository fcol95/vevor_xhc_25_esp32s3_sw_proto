#include "temp_sense.h"

#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "ntc_driver.h"

#include "modbus_params.h" // Move management of get/set params to "modbus_master" code?

static const char *LOG_TAG = "temp_sense";

// TODO: Add filtering to temp sense!
// TODO: Add sensor faults to temp sense (failed open/short NTC)!

#define TEMP_SENSE_DEFAULT_PERIOD_MS 100U
#define TEMP_SENSE_MUTEX_TIMEOUT_MS 10U

static ntc_config_t ntc_config = {
    .b_value = 3435,
    .r25_ohm = 10000,
    .fixed_ohm = 10000,
    .vdd_mv = 3300,
    .circuit_mode = CIRCUIT_MODE_NTC_GND,
    .atten = ADC_ATTEN_DB_12,
    .channel = ADC_CHANNEL_0,
    .unit = ADC_UNIT_1,
};

static ntc_device_handle_t s_ntc = NULL;
static adc_oneshot_unit_handle_t s_adc_handle = NULL;
static float s_last_measurement_temperature_degc = NAN;
static SemaphoreHandle_t s_last_measurement_temp_mutex;

static esp_err_t set_last_measurement_temperature(float value_degc)
{
    if (s_last_measurement_temp_mutex == NULL)
    {
        ESP_LOGE(LOG_TAG, "Driver not initialized!");
        return ESP_FAIL;
    }

    if (xSemaphoreTake(s_last_measurement_temp_mutex, pdMS_TO_TICKS(TEMP_SENSE_MUTEX_TIMEOUT_MS)) == pdFALSE)
    {
        ESP_LOGW(LOG_TAG, "Failed to take current command mutex!");
        return ESP_FAIL;
    }
    s_last_measurement_temperature_degc = value_degc;
    xSemaphoreGive(s_last_measurement_temp_mutex);
    return ESP_OK;
}

esp_err_t temp_sense_init(void)
{
    esp_err_t ret = ESP_OK;
    ret = ntc_dev_create(&ntc_config, &s_ntc, &s_adc_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "ntc_dev_create failed");
        return ret;
    }
    ret = ntc_dev_get_adc_handle(s_ntc, &s_adc_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "ntc_dev_get_adc_handle failed");
        return ret;
    }

    s_last_measurement_temp_mutex = xSemaphoreCreateMutex();
    if (s_last_measurement_temp_mutex == NULL)
    {
        ESP_LOGE(LOG_TAG, "Failed to create last measurement mutex!");
        return ESP_FAIL;
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
        // Get s_ntc sensor data
        float ntc_temperature_degc = NAN;
        esp_err_t ret_meas = ntc_dev_get_temperature(s_ntc, &ntc_temperature_degc);
        if (ret_meas == ESP_OK)
        {
            ESP_LOGV(LOG_TAG, "s_ntc temperature = %.2f°C.", ntc_temperature_degc);
        }
        else
        {
            ESP_LOGE(LOG_TAG, "Failed to get sensor data");
        }

        if (ret_meas == ESP_OK)
        {
            // Refresh modbus registers
            ret_esp = modbus_params_set_input_register_float(ACTUAL_TEMP_DEGC, ntc_temperature_degc);
            if (ret_esp != ESP_OK)
            {
                ESP_LOGE(LOG_TAG, "Failed to update modbus parameter ACTUAL_TEMP_DEGC!");
            }
            ret_esp = set_last_measurement_temperature(ntc_temperature_degc);
            if (ret_esp != ESP_OK)
            {
                ESP_LOGE(LOG_TAG, "Failed to update last measurement temperature data!");
            }
        }

        // Update task period if requested
        {
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
        }

        vTaskDelay(pdMS_TO_TICKS(task_period_ms));
    }
    ESP_ERROR_CHECK(ntc_dev_delete(s_ntc));
}

esp_err_t temp_sense_get_last_measurement_temperature(float *const temperature_degc)
{
    if (temperature_degc == NULL)
    {
        ESP_LOGE(LOG_TAG, "Command pointer null!");
        return ESP_FAIL;
    }
    if (s_last_measurement_temp_mutex == NULL)
    {
        ESP_LOGE(LOG_TAG, "Driver not initialized!");
        return ESP_FAIL;
    }

    if (xSemaphoreTake(s_last_measurement_temp_mutex, pdMS_TO_TICKS(TEMP_SENSE_MUTEX_TIMEOUT_MS)) == pdFALSE)
    {
        ESP_LOGW(LOG_TAG, "Failed to take requested command mutex!");
        return ESP_FAIL;
    }
    *temperature_degc = s_last_measurement_temperature_degc;
    xSemaphoreGive(s_last_measurement_temp_mutex);
    return ESP_OK;
}
