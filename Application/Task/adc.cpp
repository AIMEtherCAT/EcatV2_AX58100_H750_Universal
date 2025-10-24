//
// Created by Hang XU on 22/10/2025.
//
#include "buffer_manager.hpp"
#include "peripheral_manager.hpp"
#include "task_defs.hpp"

namespace aim::ecat::task::adc {
    ADC::ADC(buffer::Buffer *buffer) : CustomRunnable(false) {
        init_peripheral(peripheral::Type::PERIPHERAL_ADC1);

        coefficient_[0] = buffer->read_float();
        coefficient_[1] = buffer->read_float();
    }

    void ADC::write_to_master(buffer::Buffer *slave_to_master_buf) {
        parsed_adc_value_channel1.set(lowpass_filter(
            parsed_adc_value_channel1.get(),
            static_cast<float>(buffer::get_buffer(buffer::Type::ADC1_RECV)->get_buf_pointer<uint16_t>()[0]) *
            coefficient_[0],
            ADC_LF_ALPHA
        ));
        parsed_adc_value_channel2.set(lowpass_filter(
            parsed_adc_value_channel2.get(),
            static_cast<float>(buffer::get_buffer(buffer::Type::ADC1_RECV)->get_buf_pointer<uint16_t>()[1]) *
            coefficient_[1],
            ADC_LF_ALPHA
        ));

        slave_to_master_buf->write_float(parsed_adc_value_channel1.get());
        slave_to_master_buf->write_float(parsed_adc_value_channel2.get());
    }
}