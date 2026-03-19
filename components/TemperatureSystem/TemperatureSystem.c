#include "TemperatureSystem.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "pt1000_model.h"
#include "temperature_adc_port.h"

#ifndef TEMP_SYSTEM_DEBUG_LOG
#define TEMP_SYSTEM_DEBUG_LOG 1
#endif

#if TEMP_SYSTEM_DEBUG_LOG
#define TEMP_DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
#define TEMP_DEBUG_PRINT(...) ((void)0)
#endif

typedef struct {
    bool initialized;
    TempSystemConfig cfg;
    TempAdcPort adc;
    TempAdcChannel meas;
    TempAdcChannel ref;
    TempAdcChannel bus;
} TempSystemContext;

static TempSystemContext s_temp = {0};

static float temp_to_unit(float t_c, TempSystemTemperatureUnit unit)
{
    if (unit == TEMP_SYSTEM_TEMP_UNIT_FAHRENHEIT) {
        return (t_c * 9.0f / 5.0f) + 32.0f;
    }
    if (unit == TEMP_SYSTEM_TEMP_UNIT_KELVIN) {
        return t_c + 273.15f;
    }
    return t_c;
}

static float temp_compute_resistance(const TempSystemSample *sample, const TempSystemConfig *cfg)
{
    if (cfg->topology == TEMP_SYSTEM_TOPOLOGY_MEAS_REF_RATIO) {
        return cfg->reference_resistance_ohms * (sample->v_meas / sample->v_ref);
    }
    return cfg->reference_resistance_ohms * (sample->v_meas / (sample->v_bus - sample->v_meas));
}

void TempSystem_DefaultConfig(TempSystemConfig *cfg)
{
    if (cfg == NULL) {
        return;
    }

    memset(cfg, 0, sizeof(*cfg));
    cfg->gpio_meas = TEMP_SYSTEM_UNUSED_GPIO;
    cfg->gpio_ref = TEMP_SYSTEM_UNUSED_GPIO;
    cfg->gpio_bus = TEMP_SYSTEM_UNUSED_GPIO;
    cfg->reference_resistance_ohms = 1000.0f;
    cfg->topology = TEMP_SYSTEM_TOPOLOGY_MEAS_REF_RATIO;
    cfg->atten = ADC_ATTEN_DB_12;
    cfg->bitwidth = ADC_BITWIDTH_DEFAULT;
    cfg->samples = 8;
    cfg->use_adc_calibration = true;
    cfg->model = PT1000_DefaultModel();
    cfg->temperature_gain = 1.0f;
    cfg->temperature_offset_c = 0.0f;
}

esp_err_t TempSystem_Init(const TempSystemConfig *cfg)
{
    TempSystemConfig local = {0};
    esp_err_t err = ESP_OK;

    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    TempSystem_Deinit();
    local = *cfg;

    if (local.samples == 0) {
        local.samples = 1;
    }
    if (local.temperature_gain == 0.0f) {
        local.temperature_gain = 1.0f;
    }
    if (local.model.r0_ohms <= 0.0f) {
        local.model = PT1000_DefaultModel();
    }

    if (local.gpio_meas < 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (local.topology == TEMP_SYSTEM_TOPOLOGY_MEAS_REF_RATIO && local.gpio_ref < 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (local.topology == TEMP_SYSTEM_TOPOLOGY_MEAS_BUS_DIVIDER && local.gpio_bus < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    err = TempAdcPort_Init(&s_temp.adc, local.atten, local.bitwidth, local.use_adc_calibration);
    if (err != ESP_OK) {
        return err;
    }
    err = TempAdcPort_AttachGpio(&s_temp.adc, local.gpio_meas, &s_temp.meas);
    if (err != ESP_OK) {
        TempSystem_Deinit();
        return err;
    }
    err = TempAdcPort_AttachGpio(&s_temp.adc, local.gpio_ref, &s_temp.ref);
    if (err != ESP_OK) {
        TempSystem_Deinit();
        return err;
    }
    err = TempAdcPort_AttachGpio(&s_temp.adc, local.gpio_bus, &s_temp.bus);
    if (err != ESP_OK) {
        TempSystem_Deinit();
        return err;
    }

    s_temp.cfg = local;
    s_temp.initialized = true;
    return ESP_OK;
}

void TempSystem_Deinit(void)
{
    TempAdcChannel channels[3];

    channels[0] = s_temp.meas;
    channels[1] = s_temp.ref;
    channels[2] = s_temp.bus;

    TempAdcPort_Deinit(&s_temp.adc, channels, 3);
    memset(&s_temp, 0, sizeof(s_temp));
}

esp_err_t TempSystem_ReadSample(TempSystemSample *sample)
{
    TempAdcSample adc_sample = {0};
    esp_err_t err = ESP_OK;

    if (!s_temp.initialized || sample == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(sample, 0, sizeof(*sample));

    err = TempAdcPort_Read(&s_temp.adc, &s_temp.meas, s_temp.cfg.samples, &adc_sample);
    if (err != ESP_OK) {
        return err;
    }
    sample->raw_meas = adc_sample.raw;
    sample->v_meas = adc_sample.voltage;

    if (s_temp.ref.valid) {
        err = TempAdcPort_Read(&s_temp.adc, &s_temp.ref, s_temp.cfg.samples, &adc_sample);
        if (err != ESP_OK) {
            return err;
        }
        sample->raw_ref = adc_sample.raw;
        sample->v_ref = adc_sample.voltage;
    }

    if (s_temp.bus.valid) {
        err = TempAdcPort_Read(&s_temp.adc, &s_temp.bus, s_temp.cfg.samples, &adc_sample);
        if (err != ESP_OK) {
            return err;
        }
        sample->raw_bus = adc_sample.raw;
        sample->v_bus = adc_sample.voltage;
    }

    if (s_temp.cfg.topology == TEMP_SYSTEM_TOPOLOGY_MEAS_REF_RATIO && fabsf(sample->v_ref) < 1e-6f) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (s_temp.cfg.topology == TEMP_SYSTEM_TOPOLOGY_MEAS_BUS_DIVIDER && fabsf(sample->v_bus - sample->v_meas) < 1e-6f) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    sample->r_rtd_ohms = temp_compute_resistance(sample, &s_temp.cfg);
    if (!isfinite(sample->r_rtd_ohms) || sample->r_rtd_ohms <= 0.0f) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    sample->temperature_c = PT1000_TemperatureFromResistance(sample->r_rtd_ohms, &s_temp.cfg.model);
    sample->temperature_c = sample->temperature_c * s_temp.cfg.temperature_gain + s_temp.cfg.temperature_offset_c;
    if (!isfinite(sample->temperature_c)) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    TEMP_DEBUG_PRINT(
        "DEBUG: Meas: raw=%u, v=%.3f V; Ref: raw=%u, v=%.3f V; Bus: raw=%u, v=%.3f V; R=%.2f ohms; T=%.2f C\n",
        sample->raw_meas,
        sample->v_meas,
        sample->raw_ref,
        sample->v_ref,
        sample->raw_bus,
        sample->v_bus,
        sample->r_rtd_ohms,
        sample->temperature_c);

    return ESP_OK;
}

esp_err_t TempSystem_ReadCelsius(float *temperature_c)
{
    TempSystemSample sample = {0};
    esp_err_t err = ESP_OK;

    if (temperature_c == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    err = TempSystem_ReadSample(&sample);
    if (err != ESP_OK) {
        return err;
    }
    *temperature_c = sample.temperature_c;
    return ESP_OK;
}

esp_err_t TempSystem_Read(float *temperature, TempSystemTemperatureUnit unit)
{
    float t_c = 0.0f;
    esp_err_t err = ESP_OK;

    if (temperature == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    err = TempSystem_ReadCelsius(&t_c);
    if (err != ESP_OK) {
        return err;
    }
    *temperature = temp_to_unit(t_c, unit);
    return ESP_OK;
}

size_t TempSystem_GetUsedPins(int *pins, size_t max_pins)
{
    size_t count = 0;

    if (!s_temp.initialized) {
        return 0;
    }

    if (s_temp.meas.valid) {
        if (pins != NULL && count < max_pins) {
            pins[count] = s_temp.meas.gpio;
        }
        ++count;
    }
    if (s_temp.ref.valid) {
        if (pins != NULL && count < max_pins) {
            pins[count] = s_temp.ref.gpio;
        }
        ++count;
    }
    if (s_temp.bus.valid) {
        if (pins != NULL && count < max_pins) {
            pins[count] = s_temp.bus.gpio;
        }
        ++count;
    }

    return count;
}
