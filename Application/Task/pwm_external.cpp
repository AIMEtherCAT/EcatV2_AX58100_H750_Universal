//
// Created by Hang XU on 22/10/2025.
//
#include <cfloat>

#include "buffer_manager.hpp"
#include "peripheral_manager.hpp"
#include "task_defs.hpp"

namespace aim::ecat::task::pwm {

    PWM_EXTERNAL::PWM_EXTERNAL(buffer::Buffer *buffer) {
        period = 1;
        switch (buffer->read_uint8()) {
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

        expected_period_ = buffer->read_uint16();
        enabled_channel_count_ = buffer->read_uint8();
        const uint16_t init_value = buffer->read_uint16();
        control_packet.expected_period = expected_period_;
        for (int i = 0 ; i < enabled_channel_count_ ; i ++) {
            control_packet.servo_cmd[i] = init_value;
        }
        uint8_t cmd_buf[37] = {};
        memcpy(cmd_buf, &control_packet, 37);
        algorithm::crc16::append_CRC16_check_sum(cmd_buf, 37);
        get_peripheral<peripheral::UartPeripheral>()->send_by_dma(cmd_buf, 37);
        last_send_time_.set_current();
    }

    void PWM_EXTERNAL::read_from_master(buffer::Buffer *master_to_slave_buf) {
        for (int i = 0; i < enabled_channel_count_; i++) {
            control_packet.servo_cmd[i] = master_to_slave_buf->read_uint16();
        }
        uint8_t cmd_buf[37] = {};
        memcpy(cmd_buf, &control_packet, 37);
        algorithm::crc16::append_CRC16_check_sum(cmd_buf, 37);
        // if not busy, then return true, means data sent
        if (get_peripheral<peripheral::UartPeripheral>()->send_by_dma(cmd_buf, 37)) {
            last_send_time_.set_current();
        }
    }

    void PWM_EXTERNAL::uart_err() {
        get_peripheral<peripheral::UartPeripheral>()->send_by_dma(cmd_buf, 37);
    }

    void PWM_EXTERNAL::uart_dma_tx_finished_callback() {
        last_send_finished_time_.set_current();
    }

    void PWM_EXTERNAL::run_task() {
        if (last_send_finished_time_.get() < last_send_time_.get()) {
            if (HAL_GetTick() - last_send_time_.get() > 50) {
                // some packet loss happened, resend
            }
        }
    }





}