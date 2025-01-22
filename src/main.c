#include <stdio.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h" //< For LED toggling

#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_log.h"

#include "modbus_server.h"
#include "peltier_driver.h"
#include "temp_control.h"
#include "temp_sense.h"

static const char *LOG_TAG = "main";

#ifdef CONFIG_BLINK_GPIO
    #define BLINK_GPIO (gpio_num_t) CONFIG_BLINK_GPIO
#else
    #define BLINK_GPIO (gpio_num_t)21 // LED_BUILT_IN
#endif

void blink_task(void *pvParameter)
{
    // Set the GPIO as a push/pull output
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    while (1)
    {
        // Blink off (output low)
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
        // Blink on (output high)
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void print_board_info(void)
{
    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t        flash_size;
    esp_chip_info(&chip_info);
    ESP_LOGI(LOG_TAG,
             "  This is %s chip with %d CPU core(s), WiFi%s%s, ",
             CONFIG_IDF_TARGET,
             chip_info.cores,
             (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
             (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    ESP_LOGI(LOG_TAG, "silicon revision v%d.%d, ", major_rev, minor_rev);
    if (esp_flash_get_size(NULL, &flash_size) != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Get flash size failed!");
        return;
    }

    ESP_LOGI(LOG_TAG,
             "%ldMB %s flash",
             flash_size / (1024 * 1024),
             (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    ESP_LOGI(LOG_TAG, "Minimum free heap size: %ld bytes", esp_get_minimum_free_heap_size());
}

void app_main()
{
    // Set UART log level
    esp_log_level_set(LOG_TAG, ESP_LOG_INFO);

    ESP_LOGI(LOG_TAG, "-- XIAO ESP32S3 Vevor XHC-25 Modbus Controller --");

    print_board_info();

    ESP_LOGI(LOG_TAG, "Starting program...");

    // Drivers Init
    esp_err_t temp_sense_ret = temp_sense_init();

    esp_err_t peltier_driver_ret = peltier_driver_init();

    esp_err_t modbus_server_ret = modbus_server_init();

    esp_err_t temp_control_ret = temp_control_init();

    // Tasks Init
    xTaskCreate(&blink_task, "blink_task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
    if (temp_sense_ret == ESP_OK)
    {
        xTaskCreate(&temp_sense_task, "temp_sense_task", configMINIMAL_STACK_SIZE * 2, NULL, 5, NULL);
    }
    else
    {
        ESP_LOGE(LOG_TAG, "Failed to initialize temperature sensor!");
    }
    if (modbus_server_ret == ESP_OK)
    {
        xTaskCreate(&modbus_server_task, "modbus_server_task", configMINIMAL_STACK_SIZE * 2, NULL, 5, NULL);
    }
    else
    {
        ESP_LOGE(LOG_TAG, "Failed to initialize Modbus server!");
    }
    if (peltier_driver_ret == ESP_OK)
    {
        xTaskCreate(&peltier_driver_task, "peltier_driver_task", configMINIMAL_STACK_SIZE * 2, NULL, 5, NULL);
    }
    else
    {
        ESP_LOGE(LOG_TAG, "Failed to initialize Peltier driver!");
    }
    if (peltier_driver_ret == ESP_OK && temp_sense_ret == ESP_OK && temp_control_ret == ESP_OK)
    {
        xTaskCreate(&temp_control_task, "temp_control_task", configMINIMAL_STACK_SIZE * 2, NULL, 5, NULL);
    }
    else
    {
        ESP_LOGE(LOG_TAG, "Failed to initialize Temp Controller!");
    }
}