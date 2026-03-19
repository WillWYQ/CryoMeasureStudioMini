#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "TemperatureSystem.h"
#include "esp_err.h"

#define TEMP_GPIO_MEAS 20
#define TEMP_GPIO_BUS  21
#define TEMP_READ_INTERVAL_MS 1000

void app_main(void)
{
    TempSystemConfig cfg;
    esp_err_t err = ESP_OK;

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

    while (1) {
        float temperature_c = 0.0f;

        err = TempSystem_ReadCelsius(&temperature_c);
        if (err == ESP_OK) {
            printf("Temperature: %.2f C\n", temperature_c);
        } else {
            printf("TempSystem_ReadCelsius failed: %s\n", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(TEMP_READ_INTERVAL_MS));
    }
}
