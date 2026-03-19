# TemperatureSystem (Flat + Direct)

## Design Goal

This component is intentionally simple:

- Flat flow: `ADC read -> voltage -> RTD resistance -> PT1000 temperature`
- Componentized internals:
  - `temperature_adc_port.*`: only ADC oneshot + optional ADC calibration
  - `pt1000_model.*`: only PT1000 model math
  - `TemperatureSystem.c`: orchestration only
- Minimal public API, ESP-IDF style return (`esp_err_t`)

The implementation follows ESP-IDF ADC oneshot usage:

1. `adc_oneshot_new_unit()`
2. `adc_oneshot_config_channel()`
3. `adc_oneshot_read()`
4. optional `adc_cali_raw_to_voltage()`
5. `adc_oneshot_del_unit()`

Reference:

- https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32p4/api-reference/peripherals/adc/index.html
- https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32p4/api-reference/peripherals/adc/adc_oneshot.html

## Public API

```c
void TempSystem_DefaultConfig(TempSystemConfig *cfg);

esp_err_t TempSystem_Init(const TempSystemConfig *cfg);
void TempSystem_Deinit(void);

esp_err_t TempSystem_ReadSample(TempSystemSample *sample);
esp_err_t TempSystem_ReadCelsius(float *temperature_c);
esp_err_t TempSystem_Read(float *temperature, TempSystemTemperatureUnit unit);

size_t TempSystem_GetUsedPins(int *pins, size_t max_pins);
```

## Config Notes

- `topology = TEMP_SYSTEM_TOPOLOGY_MEAS_REF_RATIO`
  - uses `gpio_meas` and `gpio_ref`
  - formula: `Rrtd = Rref * Vmeas / Vref`

- `topology = TEMP_SYSTEM_TOPOLOGY_MEAS_BUS_DIVIDER`
  - uses `gpio_meas` and `gpio_bus`
  - formula: `Rrtd = Rref * Vmeas / (Vbus - Vmeas)`

- `samples` is ADC averaging count.
- `use_adc_calibration` enables ADC calibration automatically when available.

## Example

```c
TempSystemConfig cfg;
TempSystem_DefaultConfig(&cfg);

cfg.gpio_meas = 16;
cfg.gpio_ref = 17;
cfg.reference_resistance_ohms = 1000.0f;
cfg.topology = TEMP_SYSTEM_TOPOLOGY_MEAS_REF_RATIO;

ESP_ERROR_CHECK(TempSystem_Init(&cfg));

float t_c = 0.0f;
ESP_ERROR_CHECK(TempSystem_ReadCelsius(&t_c));
printf("Temperature: %.2f C\n", t_c);
```
