#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "TemperatureSystem.h"
#include "display_system.h"
#include "esp_err.h"

#define TEMP_GPIO_MEAS 20
#define TEMP_GPIO_BUS  21
#define TEMP_READ_INTERVAL_MS 1000
#define APP_LOOP_INTERVAL_MS 50

void app_main(void)
{
    TempSystemConfig cfg;
    esp_err_t err = ESP_OK;
    float latest_temperature_c = 0.0f;
    bool temperature_valid = false;
    TickType_t last_temp_tick = 0;

    TempSystem_DefaultConfig(&cfg);
    cfg.gpio_meas = TEMP_GPIO_MEAS;
    cfg.gpio_ref = TEMP_SYSTEM_UNUSED_GPIO;
    cfg.gpio_bus = TEMP_GPIO_BUS;
    cfg.topology = TEMP_SYSTEM_TOPOLOGY_MEAS_BUS_DIVIDER;
    cfg.reference_resistance_ohms = 470.0f;
    cfg.atten = ADC_ATTEN_DB_12;
    cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
    cfg.samples = 8;

    err = TempSystem_Init(&cfg);
    if (err != ESP_OK) {
        printf("TempSystem_Init failed: %s\n", esp_err_to_name(err));
        return;
    }

    err = DisplaySystem_Init();
    if (err != ESP_OK) {
        printf("DisplaySystem_Init failed: %s\n", esp_err_to_name(err));
        return;
    }

    last_temp_tick = xTaskGetTickCount() - pdMS_TO_TICKS(TEMP_READ_INTERVAL_MS);

    while (1) {
        TickType_t now = xTaskGetTickCount();

        if ((now - last_temp_tick) >= pdMS_TO_TICKS(TEMP_READ_INTERVAL_MS)) {
            err = TempSystem_ReadCelsius(&latest_temperature_c);
            if (err == ESP_OK) {
                temperature_valid = true;
                printf("Temperature: %.2f C\n", latest_temperature_c);
            } else {
                temperature_valid = false;
                printf("TempSystem_ReadCelsius failed: %s\n", esp_err_to_name(err));
            }
            last_temp_tick = now;
        }

        err = DisplaySystem_Update(latest_temperature_c, temperature_valid);
        if (err != ESP_OK) {
            printf("DisplaySystem_Update failed: %s\n", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(APP_LOOP_INTERVAL_MS));
    }
}
