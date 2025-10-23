//
// Created by Hang XU on 30/06/2025.
//
#include "adc_processor.h"
#include "io_utils.hpp"
#include "../task_defs.h"
#include "../task_manager.h"

extern "C" {
#include "adc.h"
#include "peripheral_manager.hpp"
#include "lowpass_filter.h"
}

float lowpass_filter(float prev, float current, float alpha) {
    return alpha * current + (1.0f - alpha) * prev;
}

App_ADC::App_ADC(uint8_t *args, int *offset) {
    init_adc1();
    start_adc();
    coefficient[0] = read_float(args, offset);
    coefficient[1] = read_float(args, offset);
}

void App_ADC::collect_outputs(uint8_t *input, int *input_offset) {
    parsed_adc_value[0] = lowpass_filter(parsed_adc_value[0], get_channel_value(0) * coefficient[0], 0.075);
    parsed_adc_value[1] = lowpass_filter(parsed_adc_value[1], get_channel_value(1) * coefficient[1], 0.075);
    write_float(parsed_adc_value[0], input, input_offset);
    write_float(parsed_adc_value[1], input, input_offset);
}

void App_ADC::exit() {
    HAL_ADC_Stop_DMA(&hadc1);
    deinit_adc1();
}
