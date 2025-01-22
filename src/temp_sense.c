#include "temp_sense.h"

#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "ntc_driver.h"
#include "esp_dsp.h"
#include "dsps_biquad_gen.h"

#include "modbus_params.h" // Move management of get/set params to "modbus_master" code?

static const char *LOG_TAG = "temp_sense";

// TODO: Add filtering to temp sense!
// TODO: Add sensor faults to temp sense (failed open/short NTC)!

#define TEMP_SENSE_DEFAULT_PERIOD_MS 10U
#define TEMP_SENSE_MUTEX_TIMEOUT_MS 10U

#define TEMP_SENSE_LPF_IIR_CUT_OFF_FREQ_HZ 0.01f
#define TEMP_SENSE_LPF_IIR_Q_FACTOR 0.707f // No gain >*1

static_assert(TEMP_SENSE_DEFAULT_PERIOD_MS < (uint16_t)((1.0f / TEMP_SENSE_LPF_IIR_CUT_OFF_FREQ_HZ) / 2));

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
static SemaphoreHandle_t s_last_measurement_temp_mutex = NULL;

static float lpf_iir_coeffs[5] = {0};      // Coefficients for 2nd order IIR LPF, b0, b1, b2, a1, a2
static float lpf_iir_delay_lines[2] = {0}; // Delay lines buffer for 2nd order IIR LPF

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

static inline float get_normalized_cutoff_frequency(float cutoff_frequency_hz, uint16_t sampling_rate_ms)
{
    return cutoff_frequency_hz / (1.0f / (float)sampling_rate_ms);
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

    float normalized_cutoff_freq = get_normalized_cutoff_frequency(TEMP_SENSE_LPF_IIR_CUT_OFF_FREQ_HZ, TEMP_SENSE_DEFAULT_PERIOD_MS);
    ret = dsps_biquad_gen_lpf_f32(lpf_iir_coeffs, normalized_cutoff_freq, TEMP_SENSE_LPF_IIR_Q_FACTOR);
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "dsps_biquad_gen_lpf_f32 failed");
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
            ESP_LOGV(LOG_TAG, "ntc_temperature_degc = %.2f°C.", ntc_temperature_degc);
        }
        else
        {
            ESP_LOGE(LOG_TAG, "Failed to get sensor data");
        }

        // Filter data
        float filt_ntc_temperature_degc = NAN;
        ret_meas = dsps_biquad_f32_aes3(&ntc_temperature_degc, &filt_ntc_temperature_degc, 1, lpf_iir_coeffs, lpf_iir_delay_lines); // Using the _aes3 version optimized for ESP32S3
        if (ret_meas == ESP_OK)
        {
            ESP_LOGV(LOG_TAG, "filt_ntc_temperature_degc = %.2f°C.", filt_ntc_temperature_degc);
        }
        else
        {
            ESP_LOGE(LOG_TAG, "Failed to get sensor data");
        }

        if (ret_meas == ESP_OK)
        {
            // Refresh modbus registers
            ret_esp = modbus_params_set_input_register_float(ACTUAL_TEMP_DEGC, filt_ntc_temperature_degc);
            if (ret_esp != ESP_OK)
            {
                ESP_LOGE(LOG_TAG, "Failed to update modbus parameter ACTUAL_TEMP_DEGC!");
            }
            ret_esp = set_last_measurement_temperature(filt_ntc_temperature_degc);
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
            else if (new_task_period_ms == 0 || new_task_period_ms >= (uint16_t)((1.0f / TEMP_SENSE_LPF_IIR_CUT_OFF_FREQ_HZ) / 2))
            {
                ESP_LOGE(LOG_TAG, "New measurement period too high for LPF IIR (got %d)!", new_task_period_ms);
            }
            else if (task_period_ms != new_task_period_ms)
            {
                const float new_normalized_cutoff_freq_hz = get_normalized_cutoff_frequency(TEMP_SENSE_LPF_IIR_CUT_OFF_FREQ_HZ, new_task_period_ms);
                ret_esp = dsps_biquad_gen_lpf_f32(lpf_iir_coeffs, new_normalized_cutoff_freq_hz, TEMP_SENSE_LPF_IIR_Q_FACTOR);
                if (ret_esp != ESP_OK)
                {
                    ESP_LOGE(LOG_TAG, "Failed to get modbus parameter TEMP_SENSE_PERIOD_MS!");
                }
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
