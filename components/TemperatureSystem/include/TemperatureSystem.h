#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "hal/adc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TEMP_SYSTEM_UNUSED_GPIO (-1)

typedef enum {
    TEMP_SYSTEM_TOPOLOGY_MEAS_REF_RATIO = 0,
    TEMP_SYSTEM_TOPOLOGY_MEAS_BUS_DIVIDER,
} TempSystemTopology;

typedef enum {
    TEMP_SYSTEM_TEMP_UNIT_CELSIUS = 0,
    TEMP_SYSTEM_TEMP_UNIT_FAHRENHEIT,
    TEMP_SYSTEM_TEMP_UNIT_KELVIN,
} TempSystemTemperatureUnit;

typedef struct {
    float r0_ohms;
    float a;
    float b;
    float c;
} TempSystemPT1000Model;

typedef struct {
    int gpio_meas;
    int gpio_ref;
    int gpio_bus;
    float reference_resistance_ohms;
    TempSystemTopology topology;
    adc_atten_t atten;
    adc_bitwidth_t bitwidth;
    uint8_t samples;
    bool use_adc_calibration;
    TempSystemPT1000Model model;
    float temperature_gain;
    float temperature_offset_c;
} TempSystemConfig;

typedef struct {
    uint16_t raw_meas;
    uint16_t raw_ref;
    uint16_t raw_bus;
    float v_meas;
    float v_ref;
    float v_bus;
    float r_rtd_ohms;
    float temperature_c;
} TempSystemSample;

void TempSystem_DefaultConfig(TempSystemConfig *cfg);

esp_err_t TempSystem_Init(const TempSystemConfig *cfg);
void TempSystem_Deinit(void);

esp_err_t TempSystem_ReadSample(TempSystemSample *sample);
esp_err_t TempSystem_ReadCelsius(float *temperature_c);
esp_err_t TempSystem_Read(float *temperature, TempSystemTemperatureUnit unit);

size_t TempSystem_GetUsedPins(int *pins, size_t max_pins);

#ifdef __cplusplus
}
#endif
