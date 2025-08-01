//
// Created by Hang XU on 30/06/2025.
//
#include <stdexcept>

#include "main.h"
#include "cstring"
#include "lowpass_filter.h"

extern "C" {
#include "adc.h"
}

DMA_BUFFER uint16_t adc_buf[2];

void init_adc_buf() {
    memset(adc_buf, 0, sizeof(adc_buf));
}

void start_adc() {
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adc_buf, 2);
}

float get_channel_value(int channel) {
    return static_cast<float>(adc_buf[channel]) / 65535.0f * 3.3f;
}
