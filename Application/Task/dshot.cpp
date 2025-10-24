//
// Created by Hang XU on 22/10/2025.
//
#include <cfloat>

#include "buffer_manager.hpp"
#include "peripheral_manager.hpp"
#include "task_defs.hpp"

namespace aim::ecat::task::pwm {
    DSHOT600::DSHOT600(buffer::Buffer *buffer) : CustomRunnable(false) {
        switch (buffer->read_uint8(buffer::EndianType::LITTLE)) {
            case 1: {
                tim_inst_ = &htim3;
                init_peripheral(peripheral::Type::PERIPHERAL_TIM3);
                init_dshot_dma(TIM3_FREQ);
                motor1_buffer = buffer::get_buffer(buffer::Type::DSHOT1_MOTOR1)->get_buf_pointer<uint32_t>();
                motor2_buffer = buffer::get_buffer(buffer::Type::DSHOT1_MOTOR2)->get_buf_pointer<uint32_t>();
                motor3_buffer = buffer::get_buffer(buffer::Type::DSHOT1_MOTOR3)->get_buf_pointer<uint32_t>();
                motor4_buffer = buffer::get_buffer(buffer::Type::DSHOT1_MOTOR4)->get_buf_pointer<uint32_t>();
                break;
            }
            case 2: {
                tim_inst_ = &htim2;
                init_peripheral(peripheral::Type::PERIPHERAL_TIM2);
                init_dshot_dma(TIM2_FREQ);
                motor1_buffer = buffer::get_buffer(buffer::Type::DSHOT2_MOTOR1)->get_buf_pointer<uint32_t>();
                motor2_buffer = buffer::get_buffer(buffer::Type::DSHOT2_MOTOR2)->get_buf_pointer<uint32_t>();
                motor3_buffer = buffer::get_buffer(buffer::Type::DSHOT2_MOTOR3)->get_buf_pointer<uint32_t>();
                motor4_buffer = buffer::get_buffer(buffer::Type::DSHOT2_MOTOR4)->get_buf_pointer<uint32_t>();
                break;
            }
            default: {
            }
        }

        HAL_TIM_PWM_Start(tim_inst_, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(tim_inst_, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(tim_inst_, TIM_CHANNEL_3);
        HAL_TIM_PWM_Start(tim_inst_, TIM_CHANNEL_4);

        const uint16_t init_value = buffer->read_uint16(buffer::EndianType::LITTLE);
        command_.channel1 = init_value;
        command_.channel2 = init_value;
        command_.channel3 = init_value;
        command_.channel4 = init_value;
        send_signal();
    }

    void DSHOT600::read_from_master(buffer::Buffer *master_to_slave_buf) {
        command_.channel1 = master_to_slave_buf->read_uint16(buffer::EndianType::LITTLE);
        command_.channel2 = master_to_slave_buf->read_uint16(buffer::EndianType::LITTLE);
        command_.channel3 = master_to_slave_buf->read_uint16(buffer::EndianType::LITTLE);
        command_.channel4 = master_to_slave_buf->read_uint16(buffer::EndianType::LITTLE);
        send_signal();
    }

    void DSHOT600::exit() {
        __HAL_TIM_DISABLE_DMA(tim_inst_, TIM_DMA_CC1);
        __HAL_TIM_DISABLE_DMA(tim_inst_, TIM_DMA_CC1);
        __HAL_TIM_DISABLE_DMA(tim_inst_, TIM_DMA_CC1);
        __HAL_TIM_DISABLE_DMA(tim_inst_, TIM_DMA_CC1);

        tim_inst_->hdma[TIM_DMA_ID_CC1]->XferCpltCallback = nullptr;
        tim_inst_->hdma[TIM_DMA_ID_CC2]->XferCpltCallback = nullptr;
        tim_inst_->hdma[TIM_DMA_ID_CC3]->XferCpltCallback = nullptr;
        tim_inst_->hdma[TIM_DMA_ID_CC4]->XferCpltCallback = nullptr;
    }
}