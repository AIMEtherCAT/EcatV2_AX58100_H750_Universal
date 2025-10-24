//
// Created by Hang XU on 22/10/2025.
//
#include <cfloat>

#include "buffer_manager.hpp"
#include "peripheral_manager.hpp"
#include "task_defs.hpp"

namespace aim::ecat::task::pwm {
    PWM_ONBOARD::PWM_ONBOARD(buffer::Buffer *buffer) : CustomRunnable(false) {
        uint32_t tim_freq = 0;

        switch (buffer->read_uint8(buffer::EndianType::LITTLE)) {
            case 2: {
                tim_inst_ = &htim2;
                init_peripheral(peripheral::Type::PERIPHERAL_TIM2);
                tim_freq = TIM2_FREQ;
                break;
            }
            case 3: {
                tim_inst_ = &htim3;
                init_peripheral(peripheral::Type::PERIPHERAL_TIM3);
                tim_freq = TIM3_FREQ;
                break;
            }
            default: {
            }
        }

        expected_period_ = buffer->read_uint16(buffer::EndianType::LITTLE);
        auto min_error = DBL_MAX;
        for (uint32_t psc = 0; psc <= 0xffff; ++psc) {
            const double temp_arr = static_cast<double>(expected_period_) * tim_freq / 1000000.0 / (psc + 1.0) - 1.0;
            if (temp_arr < 0 || temp_arr > 65535) {
                continue;
            }
            const double arr = temp_arr + 0.5;
            const double actual_period = static_cast<double>(psc + 1) * (arr + 1) * 1000000.0 / tim_freq;
            const double error = fabs(actual_period - expected_period_);
            if (error < min_error) {
                min_error = error;
                setting_pair_.psc = psc;
                setting_pair_.arr = static_cast<uint16_t>(arr);
            }
            if (error < 1e-6) {
                break;
            }
        }

        const uint16_t init_value = buffer->read_uint16(buffer::EndianType::LITTLE);
        command_.channel1 = calculate_compare(init_value);
        command_.channel2 = calculate_compare(init_value);
        command_.channel3 = calculate_compare(init_value);
        command_.channel4 = calculate_compare(init_value);

        __HAL_TIM_SET_PRESCALER(tim_inst_, setting_pair_.psc);
        __HAL_TIM_SET_AUTORELOAD(tim_inst_, setting_pair_.arr);

        send_signal();
        HAL_TIM_PWM_Start(tim_inst_, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(tim_inst_, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(tim_inst_, TIM_CHANNEL_3);
        HAL_TIM_PWM_Start(tim_inst_, TIM_CHANNEL_4);
    }

    void PWM_ONBOARD::read_from_master(buffer::Buffer *master_to_slave_buf) {
        command_.channel1 = calculate_compare(master_to_slave_buf->read_uint16(buffer::EndianType::LITTLE));
        command_.channel2 = calculate_compare(master_to_slave_buf->read_uint16(buffer::EndianType::LITTLE));
        command_.channel3 = calculate_compare(master_to_slave_buf->read_uint16(buffer::EndianType::LITTLE));
        command_.channel4 = calculate_compare(master_to_slave_buf->read_uint16(buffer::EndianType::LITTLE));

        send_signal();
    }
}
