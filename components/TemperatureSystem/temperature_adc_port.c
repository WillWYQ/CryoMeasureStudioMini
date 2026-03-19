#include "temperature_adc_port.h"

#include <string.h>

#include "soc/soc_caps.h"

static int temp_adc_unit_index(adc_unit_t unit)
{
    if (unit == ADC_UNIT_1) {
        return 0;
    }
    if (unit == ADC_UNIT_2) {
        return 1;
    }
    return -1;
}

static uint32_t temp_adc_full_scale_mv(adc_atten_t atten)
{
    switch (atten) {
    case ADC_ATTEN_DB_0:
        return 1100U;
    case ADC_ATTEN_DB_2_5:
        return 1500U;
    case ADC_ATTEN_DB_6:
        return 2200U;
    case ADC_ATTEN_DB_12:
    default:
        return 3900U;
    }
}

static adc_bitwidth_t temp_adc_effective_bitwidth(adc_bitwidth_t bitwidth)
{
    if (bitwidth != ADC_BITWIDTH_DEFAULT) {
        return bitwidth;
    }
#ifdef SOC_ADC_RTC_MAX_BITWIDTH
    return (adc_bitwidth_t)SOC_ADC_RTC_MAX_BITWIDTH;
#else
    return ADC_BITWIDTH_12;
#endif
}

static void temp_adc_delete_cali(TempAdcChannel *channel)
{
    if (channel == NULL || channel->cali == NULL) {
        return;
    }

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (channel->cali_scheme == ADC_CALI_SCHEME_VER_CURVE_FITTING) {
        adc_cali_delete_scheme_curve_fitting(channel->cali);
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (channel->cali_scheme == ADC_CALI_SCHEME_VER_LINE_FITTING) {
        adc_cali_delete_scheme_line_fitting(channel->cali);
    }
#endif

    channel->cali = NULL;
    channel->cali_scheme = 0;
}

static void temp_adc_try_enable_calibration(const TempAdcPort *port, TempAdcChannel *channel)
{
    adc_cali_scheme_ver_t mask = 0;

    if (!port->use_cali) {
        return;
    }
    if (adc_cali_check_scheme(&mask) != ESP_OK) {
        return;
    }

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if ((mask & ADC_CALI_SCHEME_VER_CURVE_FITTING) != 0) {
        adc_cali_curve_fitting_config_t cali_cfg = {
            .unit_id = channel->unit,
            .chan = channel->channel,
            .atten = port->atten,
            .bitwidth = port->bitwidth,
        };
        if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &channel->cali) == ESP_OK) {
            channel->cali_scheme = ADC_CALI_SCHEME_VER_CURVE_FITTING;
            return;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if ((mask & ADC_CALI_SCHEME_VER_LINE_FITTING) != 0) {
        adc_cali_line_fitting_config_t cali_cfg = {
            .unit_id = channel->unit,
            .atten = port->atten,
            .bitwidth = port->bitwidth,
        };
        if (adc_cali_create_scheme_line_fitting(&cali_cfg, &channel->cali) == ESP_OK) {
            channel->cali_scheme = ADC_CALI_SCHEME_VER_LINE_FITTING;
            return;
        }
    }
#endif
}

static esp_err_t temp_adc_read_raw_average(const TempAdcPort *port, const TempAdcChannel *channel, uint8_t samples, uint16_t *raw)
{
    uint32_t sum = 0;
    uint8_t count = samples == 0 ? 1 : samples;
    int unit_index = 0;

    if (port == NULL || channel == NULL || raw == NULL || !channel->valid) {
        return ESP_ERR_INVALID_ARG;
    }

    unit_index = temp_adc_unit_index(channel->unit);
    for (uint8_t i = 0; i < count; ++i) {
        int one = 0;
        esp_err_t err = adc_oneshot_read(port->units[unit_index], channel->channel, &one);
        if (err != ESP_OK) {
            return err;
        }
        sum += (uint32_t)one;
    }

    *raw = (uint16_t)((sum + (count / 2U)) / count);
    return ESP_OK;
}

static esp_err_t temp_adc_raw_to_voltage(const TempAdcPort *port, const TempAdcChannel *channel, uint16_t raw, float *voltage)
{
    int mv = 0;

    if (port == NULL || channel == NULL || voltage == NULL || !channel->valid) {
        return ESP_ERR_INVALID_ARG;
    }

    if (channel->cali != NULL) {
        esp_err_t err = adc_cali_raw_to_voltage(channel->cali, (int)raw, &mv);
        if (err != ESP_OK) {
            return err;
        }
    } else {
        uint32_t max_raw = (1U << (uint32_t)temp_adc_effective_bitwidth(port->bitwidth)) - 1U;
        mv = (int)((((uint32_t)raw) * temp_adc_full_scale_mv(port->atten) + (max_raw / 2U)) / max_raw);
    }

    *voltage = ((float)mv) / 1000.0f;
    return ESP_OK;
}


// ********************* Public API

esp_err_t TempAdcPort_Init(TempAdcPort *port, adc_atten_t atten, adc_bitwidth_t bitwidth, bool use_cali)
{
    if (port == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(port, 0, sizeof(*port));
    port->atten = atten;
    port->bitwidth = bitwidth;
    port->use_cali = use_cali;
    return ESP_OK;
}

void TempAdcPort_Deinit(TempAdcPort *port, TempAdcChannel *channels, size_t channel_count)
{
    size_t i = 0;

    if (channels != NULL) {
        for (i = 0; i < channel_count; ++i) {
            temp_adc_delete_cali(&channels[i]);
        }
    }

    if (port != NULL) {
        for (i = 0; i < 2; ++i) {
            if (port->unit_ready[i] && port->units[i] != NULL) {
                adc_oneshot_del_unit(port->units[i]);
            }
        }
        memset(port, 0, sizeof(*port));
    }
}

esp_err_t TempAdcPort_AttachGpio(TempAdcPort *port, int gpio, TempAdcChannel *out_channel)
{
    adc_oneshot_chan_cfg_t chan_cfg = {0};
    adc_oneshot_unit_init_cfg_t unit_cfg = {0};
    int unit_index = 0;
    esp_err_t err = ESP_OK;

    if (port == NULL || out_channel == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(out_channel, 0, sizeof(*out_channel));
    if (gpio < 0) {
        return ESP_OK;
    }

    err = adc_oneshot_io_to_channel(gpio, &out_channel->unit, &out_channel->channel);
    if (err != ESP_OK) {
        return err;
    }

    unit_index = temp_adc_unit_index(out_channel->unit);
    if (unit_index < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!port->unit_ready[unit_index]) {
        unit_cfg.unit_id = out_channel->unit;
        unit_cfg.ulp_mode = ADC_ULP_MODE_DISABLE;
        err = adc_oneshot_new_unit(&unit_cfg, &port->units[unit_index]);
        if (err != ESP_OK) {
            return err;
        }
        port->unit_ready[unit_index] = true;
    }

    chan_cfg.atten = port->atten;
    chan_cfg.bitwidth = port->bitwidth;
    err = adc_oneshot_config_channel(port->units[unit_index], out_channel->channel, &chan_cfg);
    if (err != ESP_OK) {
        return err;
    }

    out_channel->valid = true;
    out_channel->gpio = gpio;
    temp_adc_try_enable_calibration(port, out_channel);
    return ESP_OK;
}

esp_err_t TempAdcPort_Read(const TempAdcPort *port, const TempAdcChannel *channel, uint8_t samples, TempAdcSample *sample)
{
    esp_err_t err = ESP_OK;

    if (sample == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    err = temp_adc_read_raw_average(port, channel, samples, &sample->raw);
    if (err != ESP_OK) {
        return err;
    }
    err = temp_adc_raw_to_voltage(port, channel, sample->raw, &sample->voltage);
    if (err != ESP_OK) {
        return err;
    }
    return ESP_OK;
}
