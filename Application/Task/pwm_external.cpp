//
// Created by Hang XU on 22/10/2025.
//
#include "buffer_utils.hpp"
#include "peripheral_utils.hpp"
#include "task_defs.hpp"

namespace aim::ecat::task::pwm {
    PWM_EXTERNAL::PWM_EXTERNAL(buffer::Buffer *buffer) : UartRunnable(true, TaskType::EXTERNAL_PWM) {
        period = 1;

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
                init_peripheral(peripheral::Type::PERIPHERAL_USART1);
                break;
            }
            case 4: {
                init_peripheral(peripheral::Type::PERIPHERAL_UART4);
                break;
            }
            default: {
            }
        }

        expected_period_ = buffer->read_uint16(buffer::EndianType::LITTLE);
        enabled_channel_count_ = buffer->read_uint8(buffer::EndianType::LITTLE);
        init_value_ = buffer->read_uint16(buffer::EndianType::LITTLE);
        control_packet_.expected_period = expected_period_;
        for (int i = 0; i < enabled_channel_count_; i++) {
            control_packet_.servo_cmd[i] = init_value_;
        }
        send_packet();
        // set last send time to now because it is the first send
        last_send_time_.set_current();
    }

    void PWM_EXTERNAL::read_from_master(buffer::Buffer *master_to_slave_buf) {
        if (in_protection_.get()) {
            for (int i = 0; i < enabled_channel_count_; i++) {
                control_packet_.servo_cmd[i] = init_value_;
            }
        } else {
            for (int i = 0; i < enabled_channel_count_; i++) {
                control_packet_.servo_cmd[i] = master_to_slave_buf->read_uint16(buffer::EndianType::LITTLE);
            }
        }

        send_packet();
    }

    void PWM_EXTERNAL::uart_err() {
        get_peripheral<peripheral::UartPeripheral>()->reset_tx_dma();
        send_packet();
    }

    void PWM_EXTERNAL::uart_dma_tx_finished_callback() {
        last_send_finished_time_.set_current();
    }

    void PWM_EXTERNAL::run_task() {
        if (last_send_finished_time_.get() < last_send_time_.get()) {
            if (HAL_GetTick() - last_send_time_.get() > 50) {
                // some packet loss happened, reset and then resend
                uart_err();
            }
        }
    }

    void PWM_EXTERNAL::on_connection_lost() {
        if (connection_lost_action_ == ConnectionLostAction::RESET_TO_DEFAULT) {
            in_protection_.set();

            for (int i = 0; i < enabled_channel_count_; i++) {
                control_packet_.servo_cmd[i] = init_value_;
            }
        }

        send_packet();
    }

    void PWM_EXTERNAL::on_connection_recover() {
        in_protection_.clear();
    }
}
