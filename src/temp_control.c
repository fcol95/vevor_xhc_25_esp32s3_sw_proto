#include <math.h>

#include "modbus_params.h" // Move management of get/set params to "modbus_master" code?
#include "peltier_driver.h"
#include "temp_control.h"
#include "temp_sense.h"

static const char *LOG_TAG = "temp_control";

#define TEMP_CONTROLLER_PERIOD_MS                                100
#define TEMP_CONTROL_MAX_ERROR_COUNT                             5 // TODO: Replace with more complex logic?

#define TEMP_CONTROLLER_REQUEST_MAX_TEMP_DEGC                    45.0f
#define TEMP_CONTROLLER_REQUEST_MIN_TEMP_DEGC                    0.0f

#define TEMP_CONTROLLER_SENSE_ABORT_MAX_TEMP_DEGC                (TEMP_CONTROLLER_REQUEST_MAX_TEMP_DEGC + 5.0f)
#define TEMP_CONTROLLER_SENSE_ABORT_MIN_TEMP_DEGC                (TEMP_CONTROLLER_REQUEST_MIN_TEMP_DEGC - 5.0f)
#define TEMP_CONTROLLER_HYSTERESIS_DEGC                          0.5f
#define TEMP_CONTROLLER_MINIMUM_ELAPSED_TIME_BETWEEN_COMMANDS_MS 2000U

static_assert(TEMP_CONTROLLER_REQUEST_MAX_TEMP_DEGC > TEMP_CONTROLLER_REQUEST_MIN_TEMP_DEGC,
              "Requested max temperature must be greater than min temperature!");
static_assert(TEMP_CONTROLLER_SENSE_ABORT_MAX_TEMP_DEGC > TEMP_CONTROLLER_SENSE_ABORT_MIN_TEMP_DEGC,
              "Abort max temperature must be greater than min temperature!");
static_assert(TEMP_CONTROLLER_SENSE_ABORT_MAX_TEMP_DEGC > TEMP_CONTROLLER_REQUEST_MAX_TEMP_DEGC,
              "Sense max temperature must be greater than requested max temperature!");

static PeltierDriver_Command_t s_last_known_peltier_command = PELTIER_DRIVER_COMMAND_NONE;

static TickType_t get_elapsed_time_ms(TickType_t last_time_ms)
{
    TickType_t current_time_ms = pdTICKS_TO_MS(xTaskGetTickCount());
    TickType_t elapsed_time_ms = current_time_ms - pdMS_TO_TICKS(last_time_ms);
    return elapsed_time_ms;
}

esp_err_t temp_control_init(void) { return ESP_OK; }

esp_err_t temp_controller_execute(float requested_temp_degc)
{
    esp_err_t         ret = ESP_OK;
    bool              abort_control = false;
    static bool       first_run = true;
    static TickType_t last_command_change_time_ms = 0;

    // Getting actual temperature
    float actual_temp_degc = NAN;
    ret = temp_sense_get_last_measurement_temperature(&actual_temp_degc);
    if (ret != ESP_OK || !finitef(actual_temp_degc))
    {
        ESP_LOGE(LOG_TAG, "Can't get a valid actual temperature measurement (%.2f)!", actual_temp_degc);
        return ret;
    }

    // Checking abort conditions
    if (requested_temp_degc > TEMP_CONTROLLER_REQUEST_MAX_TEMP_DEGC
        || requested_temp_degc < TEMP_CONTROLLER_REQUEST_MIN_TEMP_DEGC)
    {
        ESP_LOGE(LOG_TAG,
                 "Requested temperature out of bounds (%.2fdegC)! Aborting temperature control!",
                 requested_temp_degc);
        abort_control = true;
    }
    if (actual_temp_degc > TEMP_CONTROLLER_SENSE_ABORT_MAX_TEMP_DEGC
        || actual_temp_degc < TEMP_CONTROLLER_SENSE_ABORT_MIN_TEMP_DEGC)
    {
        ESP_LOGE(LOG_TAG,
                 "Actual temperature out of bounds (%.2fdegC)! Aborting temperature control!",
                 actual_temp_degc);
        abort_control = true;
    }
    // TODO: Replace bang-bang logic with PID controller!
    PeltierDriver_Command_t current_command = PELTIER_DRIVER_COMMAND_NONE;
    ret = peltier_driver_get_current_command(&current_command);
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Can't get a the actual peltier command!");
        return ret;
    }

    float delta_temp_degc = requested_temp_degc - actual_temp_degc;
    if (delta_temp_degc >= TEMP_CONTROLLER_HYSTERESIS_DEGC)
    {
        if ((first_run
             || get_elapsed_time_ms(last_command_change_time_ms)
                    > TEMP_CONTROLLER_MINIMUM_ELAPSED_TIME_BETWEEN_COMMANDS_MS)
            && (s_last_known_peltier_command != PELTIER_DRIVER_COMMAND_HEAT
                || s_last_known_peltier_command != current_command))
        {
            const PeltierDriver_Command_t new_command = PELTIER_DRIVER_COMMAND_HEAT;
            ret = peltier_driver_set_requested_command(new_command);
            if (ret != ESP_OK)
            {
                ESP_LOGE(LOG_TAG, "Requested temperature delta above hysteresis, but can't heat!");
                return ret;
            }
            s_last_known_peltier_command = new_command;
            last_command_change_time_ms = pdTICKS_TO_MS(xTaskGetTickCount());
            if (first_run) first_run = false;
            ESP_LOGI(LOG_TAG,
                     "Requested temperature delta above hysteresis (%.2fdegC), starting to heat...",
                     delta_temp_degc);
        }
    }
    else if (delta_temp_degc <= -TEMP_CONTROLLER_HYSTERESIS_DEGC)
    {
        if ((first_run
             || get_elapsed_time_ms(last_command_change_time_ms)
                    > TEMP_CONTROLLER_MINIMUM_ELAPSED_TIME_BETWEEN_COMMANDS_MS)
            && (s_last_known_peltier_command != PELTIER_DRIVER_COMMAND_COOL
                || s_last_known_peltier_command != current_command))
        {
            const PeltierDriver_Command_t new_command = PELTIER_DRIVER_COMMAND_COOL;
            ret = peltier_driver_set_requested_command(new_command);
            if (ret != ESP_OK)
            {
                ESP_LOGE(LOG_TAG, "Requested temperature delta below hysteresis, but can't cool!");
                return ret;
            }
            s_last_known_peltier_command = new_command;
            last_command_change_time_ms = pdTICKS_TO_MS(xTaskGetTickCount());
            if (first_run) first_run = false;
            ESP_LOGI(LOG_TAG,
                     "Requested temperature delta below hysteresis (%.2fdegC), starting to cool...",
                     delta_temp_degc);
        }
    }
    else if ((s_last_known_peltier_command == PELTIER_DRIVER_COMMAND_COOL
              && delta_temp_degc >= TEMP_CONTROLLER_HYSTERESIS_DEGC)
             || (s_last_known_peltier_command == PELTIER_DRIVER_COMMAND_HEAT
                 && delta_temp_degc <= -TEMP_CONTROLLER_HYSTERESIS_DEGC))
    {
        if ((first_run
             || (get_elapsed_time_ms(last_command_change_time_ms)
                 > TEMP_CONTROLLER_MINIMUM_ELAPSED_TIME_BETWEEN_COMMANDS_MS))
            && (s_last_known_peltier_command != PELTIER_DRIVER_COMMAND_NONE
                || s_last_known_peltier_command != current_command))
        {
            const PeltierDriver_Command_t new_command = PELTIER_DRIVER_COMMAND_NONE;
            ret = peltier_driver_set_requested_command(PELTIER_DRIVER_COMMAND_NONE);
            if (ret != ESP_OK)
            {
                ESP_LOGE(LOG_TAG, "Requested temperature delta within hysteresis, but can't stop peltier!");
                return ret;
            }
            s_last_known_peltier_command = new_command;
            last_command_change_time_ms = pdTICKS_TO_MS(xTaskGetTickCount());
            if (first_run) first_run = false;
            ESP_LOGI(LOG_TAG,
                     "Requested temperature delta within hysteresis (%.2f), stopping peltier...",
                     delta_temp_degc);
        }
    }
    else if (abort_control)
    {
        if (s_last_known_peltier_command != PELTIER_DRIVER_COMMAND_NONE
            || s_last_known_peltier_command != current_command)
        {
            const PeltierDriver_Command_t new_command = PELTIER_DRIVER_COMMAND_NONE;
            ret = peltier_driver_set_requested_command(PELTIER_DRIVER_COMMAND_NONE);
            if (ret != ESP_OK)
            {
                ESP_LOGE(LOG_TAG, "Abort true, but can't stop peltier!");
                return ret;
            }
            s_last_known_peltier_command = new_command;
            last_command_change_time_ms = pdTICKS_TO_MS(xTaskGetTickCount());
            abort_control = false;
            ESP_LOGI(LOG_TAG, "Aborting true, stopping peltier...");
        }
    }
    return ret;
}

void temp_control_task(void *pvParameter)
{
    esp_err_t ret = ESP_OK;
    uint8_t   error_count = 0;
    while (1)
    {
        static bool last_temp_control_enabled = false;
        bool        temp_control_enabled = false;
        esp_err_t   ret_enabled = modbus_params_get_coil_state(ENABLE_TEMP_CONTROL, &temp_control_enabled);
        if (ret_enabled != ESP_OK)
        {
            ESP_LOGE(LOG_TAG, "Failed to get modbus parameter ENABLE_TEMP_CONTROL!");
            if (error_count < 0xFF) error_count++;
        }
        else
        {
            if (last_temp_control_enabled && !temp_control_enabled)
            {
                ESP_LOGI(LOG_TAG, "Disabling temperature controller!");
                last_temp_control_enabled = temp_control_enabled;
            }
            else if (!last_temp_control_enabled && temp_control_enabled)
            {
                ESP_LOGI(LOG_TAG, "Enabling temperature controller!");
                last_temp_control_enabled = temp_control_enabled;
            }
        }

        if (s_last_known_peltier_command != PELTIER_DRIVER_COMMAND_NONE
            && ((!temp_control_enabled && ret_enabled == ESP_OK) || error_count > TEMP_CONTROL_MAX_ERROR_COUNT))
        {
            if (error_count > TEMP_CONTROL_MAX_ERROR_COUNT)
            {
                ESP_LOGE(LOG_TAG, "Error count above threshold! Force disabling peltier request.");
            }
            PeltierDriver_Command_t current_command = PELTIER_DRIVER_COMMAND_NONE;
            ret = peltier_driver_get_current_command(&current_command);
            if (ret != ESP_OK || current_command != PELTIER_DRIVER_COMMAND_NONE)
            {
                ret = peltier_driver_set_requested_command(PELTIER_DRIVER_COMMAND_NONE);
                if (ret != ESP_OK)
                {
                    ESP_LOGE(LOG_TAG, "Can't disable peltier module!");
                }
                else
                {
                    s_last_known_peltier_command = PELTIER_DRIVER_COMMAND_NONE;
                    error_count = 0;
                }
            }
            else
            {
                s_last_known_peltier_command = PELTIER_DRIVER_COMMAND_NONE;
                error_count = 0;
            }
        }
        if (temp_control_enabled && ret_enabled == ESP_OK)
        {
            static bool waiting_on_valid_temp_request = true;
            float       requested_temp_degc = NAN;
            ret = modbus_params_get_holding_register_float(REQUESTED_TEMP_DEGC, &requested_temp_degc);
            if ((ret == ESP_OK) && finitef(requested_temp_degc))
            {
                if (waiting_on_valid_temp_request)
                {
                    waiting_on_valid_temp_request = false;
                    ESP_LOGI(LOG_TAG, "Starting temperature controller with request %.2fdegC!", requested_temp_degc);
                }
                esp_err_t controller_ret = temp_controller_execute(requested_temp_degc);
                if (controller_ret == ESP_OK)
                {
                    error_count = 0;
                }
                else
                {
                    if (error_count < 0xFF) error_count++;
                    ESP_LOGE(LOG_TAG,
                             "Failed to execute temp controller! Increasing error count to %d/%d!",
                             error_count,
                             TEMP_CONTROL_MAX_ERROR_COUNT);
                }
            }
            else if ((ret == ESP_OK) && (!finitef(requested_temp_degc)))
            {
                if (error_count < 0xFF) error_count++;
                if (!waiting_on_valid_temp_request)
                {
                    waiting_on_valid_temp_request = true;
                    ESP_LOGE(LOG_TAG, "Got invalid temperature request! (got %.2f)", requested_temp_degc);
                }
            }
            else // if (ret != ESP_OK)
            {
                if (error_count < 0xFF) error_count++;
                ESP_LOGE(LOG_TAG, "Failed to get temperature request from modbus parameters!");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(TEMP_CONTROLLER_PERIOD_MS));
    }
}
