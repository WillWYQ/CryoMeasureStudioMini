#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"

typedef struct {
    bool valid;
    int gpio;
    adc_unit_t unit; 
    adc_channel_t channel;
    adc_cali_handle_t cali;
    adc_cali_scheme_ver_t cali_scheme;
} TempAdcChannel;

typedef struct {
    uint16_t raw;
    float voltage;
} TempAdcSample;

typedef struct {
    adc_atten_t atten;
    adc_bitwidth_t bitwidth;
    bool use_cali;
    adc_oneshot_unit_handle_t units[2];
    bool unit_ready[2];
} TempAdcPort;

esp_err_t TempAdcPort_Init(TempAdcPort *port, adc_atten_t atten, adc_bitwidth_t bitwidth, bool use_cali);
void TempAdcPort_Deinit(TempAdcPort *port, TempAdcChannel *channels, size_t channel_count);

esp_err_t TempAdcPort_AttachGpio(TempAdcPort *port, int gpio, TempAdcChannel *out_channel);
esp_err_t TempAdcPort_Read(const TempAdcPort *port, const TempAdcChannel *channel, uint8_t samples, TempAdcSample *sample);
