//
// Created by Hang XU on 22/10/2025.
//
#include "buffer_utils.hpp"
#include "peripheral_utils.hpp"
#include "task_defs.hpp"

extern "C" {
#include "tim.h"
}

namespace aim::ecat::task::pwm {
    DSHOT600::DSHOT600(buffer::Buffer *buffer) : CustomRunnable(false, TaskType::DSHOT) {
        switch (buffer->read_uint8(buffer::EndianType::LITTLE)) {
            case 0x01: {
                connection_lost_action_ = ConnectionLostAction::KEEP_LAST;
                break;
            }
            case 0x02: {
                connection_lost_action_ = ConnectionLostAction::RESET_TO_DEFAULT;
                break;
            }
            default: {
            }
        }

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

        init_value_ = buffer->read_uint16(buffer::EndianType::LITTLE);
        command_.channel1 = init_value_;
        command_.channel2 = init_value_;
        command_.channel3 = init_value_;
        command_.channel4 = init_value_;
        send_signal();
    }

    void DSHOT600::read_from_master(buffer::Buffer *master_to_slave_buf) {
        if (in_protection_.get()) {
            command_.channel1 = init_value_;
            command_.channel2 = init_value_;
            command_.channel3 = init_value_;
            command_.channel4 = init_value_;
        } else {
            command_.channel1 = master_to_slave_buf->read_uint16(buffer::EndianType::LITTLE);
            command_.channel2 = master_to_slave_buf->read_uint16(buffer::EndianType::LITTLE);
            command_.channel3 = master_to_slave_buf->read_uint16(buffer::EndianType::LITTLE);
            command_.channel4 = master_to_slave_buf->read_uint16(buffer::EndianType::LITTLE);
        }

        send_signal();
    }

    void DSHOT600::on_connection_lost() {
        if (connection_lost_action_ == ConnectionLostAction::RESET_TO_DEFAULT) {
            in_protection_.set();
            command_.channel1 = init_value_;
            command_.channel2 = init_value_;
            command_.channel3 = init_value_;
            command_.channel4 = init_value_;
        }
        send_signal();
    }

    void DSHOT600::on_connection_recover() {
        in_protection_.clear();
    }

    void DSHOT600::on_packet_sent() const {
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
