//
// Created by Hang XU on 22/10/2025.
//
#include "buffer_utils.hpp"
#include "peripheral_utils.hpp"
#include "task_defs.hpp"

extern "C" {
#include "fdcan.h"
}

namespace aim::ecat::task::lk_motor {
    LK_MOTOR::LK_MOTOR(buffer::Buffer *buffer) : CanRunnable(true, TaskType::LK_MOTOR) {
        init_peripheral(peripheral::Type::PERIPHERAL_CAN);

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

        period = buffer->read_uint16(buffer::EndianType::LITTLE);

        switch (buffer->read_uint8(buffer::EndianType::LITTLE)) {
            case 0x01: {
                mode_ = CtrlMode::OPEN_LOOP_CURRENT;
                break;
            }
            case 0x02: {
                mode_ = CtrlMode::TORQUE;
                break;
            }
            case 0x03: {
                mode_ = CtrlMode::SPEED_WITH_TORQUE_LIMIT;
                break;
            }
            case 0x04: {
                mode_ = CtrlMode::MULTI_ROUND_POSITION;
                break;
            }
            case 0x05: {
                mode_ = CtrlMode::MULTI_ROUND_POSITION_WITH_SPEED_LIMIT;
                break;
            }
            case 0x06: {
                mode_ = CtrlMode::SINGLE_ROUND_POSITION;
                break;
            }
            case 0x07: {
                mode_ = CtrlMode::SINGLE_ROUND_POSITION_WITH_SPEED_LIMIT;
                break;
            }
            case 0x08: {
                mode_ = CtrlMode::BROADCAST_CURRENT;
                break;
            }
            default: {
            }
        }

        if (mode_ != CtrlMode::BROADCAST_CURRENT) {
            packet_id_ = buffer->read_uint32(buffer::EndianType::LITTLE);
        } else {
            packet_id_ = BROADCAST_MODE_CTRL_PACKET_ID;
        }
        can_id_type_ = FDCAN_STANDARD_ID;
        shared_tx_header_.Identifier = packet_id_;
        shared_tx_header_.IdType = FDCAN_STANDARD_ID;
        shared_tx_header_.TxFrameType = FDCAN_DATA_FRAME;
        shared_tx_header_.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        shared_tx_header_.BitRateSwitch = FDCAN_BRS_OFF;
        shared_tx_header_.FDFormat = FDCAN_CLASSIC_CAN;
        shared_tx_header_.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        shared_tx_header_.MessageMarker = 0;
        shared_tx_header_.DataLength = 8;

        switch (buffer->read_uint8(buffer::EndianType::LITTLE)) {
            case 0x01: {
                can_inst_ = &hfdcan1;
                break;
            }
            case 0x02: {
                can_inst_ = &hfdcan2;
                break;
            }
            default: {
            }
        }
    }

    void LK_MOTOR::write_to_master(buffer::Buffer *slave_to_master_buf) {
        if (mode_ != CtrlMode::BROADCAST_CURRENT) {
            // bit 0 online?
            // bit 1 querying state?
            // bit 2 enabled?
            slave_to_master_buf->write_uint8(buffer::EndianType::LITTLE,
                                             (motor_[0].is_online() ? 1 : 0)
                                             | (state_.get() == State::QUERYING_STATE ? 1 : 0) << 1
                                             | (state_.get() == State::ENABLED ? 1 : 0) << 2
            );
            slave_to_master_buf->write_int16(buffer::EndianType::LITTLE, motor_[0].report.current.get());
            slave_to_master_buf->write_int16(buffer::EndianType::LITTLE, motor_[0].report.speed.get());
            slave_to_master_buf->write_uint16(buffer::EndianType::LITTLE, motor_[0].report.ecd.get());
            slave_to_master_buf->write_uint8(buffer::EndianType::LITTLE, motor_[0].report.temperature.get());
        } else {
            for (auto &motor: motor_) {
                slave_to_master_buf->write_uint8(buffer::EndianType::LITTLE, motor.is_online() ? 1 : 0);
                slave_to_master_buf->write_int16(buffer::EndianType::LITTLE, motor.report.current.get());
                slave_to_master_buf->write_int16(buffer::EndianType::LITTLE, motor.report.speed.get());
                slave_to_master_buf->write_uint16(buffer::EndianType::LITTLE, motor.report.ecd.get());
                slave_to_master_buf->write_uint8(buffer::EndianType::LITTLE, motor.report.temperature.get());
            }
        }
    }

    void LK_MOTOR::on_connection_lost() {
        if (connection_lost_action_ == ConnectionLostAction::RESET_TO_DEFAULT) {
            for (auto &motor: motor_) {
                motor.command.is_enable.clear();
                motor.command.cmd_int16.set(0);
            }
        }
    }

    void LK_MOTOR::read_from_master(buffer::Buffer *master_to_slave_buf) {
        if (mode_ != CtrlMode::BROADCAST_CURRENT) {
            motor_[0].command.is_enable.set(master_to_slave_buf->read_uint8(buffer::EndianType::LITTLE));

            switch (mode_) {
                case CtrlMode::OPEN_LOOP_CURRENT:
                case CtrlMode::TORQUE: {
                    motor_[0].command.cmd_int16.set(master_to_slave_buf->read_int16(buffer::EndianType::LITTLE));
                    break;
                }
                case CtrlMode::SPEED_WITH_TORQUE_LIMIT: {
                    motor_[0].command.cmd_int16.set(master_to_slave_buf->read_int16(buffer::EndianType::LITTLE));
                    motor_[0].command.cmd_int32.set(master_to_slave_buf->read_int32(buffer::EndianType::LITTLE));
                    break;
                }
                case CtrlMode::MULTI_ROUND_POSITION: {
                    motor_[0].command.cmd_int32.set(master_to_slave_buf->read_int32(buffer::EndianType::LITTLE));
                    break;
                }
                case CtrlMode::MULTI_ROUND_POSITION_WITH_SPEED_LIMIT: {
                    motor_[0].command.cmd_uint16.set(master_to_slave_buf->read_uint16(buffer::EndianType::LITTLE));
                    motor_[0].command.cmd_int32.set(master_to_slave_buf->read_int32(buffer::EndianType::LITTLE));
                    break;
                }
                case CtrlMode::SINGLE_ROUND_POSITION: {
                    motor_[0].command.cmd_uint8.set(master_to_slave_buf->read_uint8(buffer::EndianType::LITTLE));
                    motor_[0].command.cmd_uint32.set(master_to_slave_buf->read_uint32(buffer::EndianType::LITTLE));
                    break;
                }
                case CtrlMode::SINGLE_ROUND_POSITION_WITH_SPEED_LIMIT: {
                    motor_[0].command.cmd_uint8.set(master_to_slave_buf->read_uint8(buffer::EndianType::LITTLE));
                    motor_[0].command.cmd_uint16.set(master_to_slave_buf->read_uint16(buffer::EndianType::LITTLE));
                    motor_[0].command.cmd_uint32.set(master_to_slave_buf->read_uint32(buffer::EndianType::LITTLE));
                    break;
                }
                default: {
                }
            }
        } else {
            for (auto &motor: motor_) {
                motor.command.cmd_int16.set(master_to_slave_buf->read_int16(buffer::EndianType::LITTLE));
            }
        }
    }

    void LK_MOTOR::can_recv(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data) {
        int index = 1;
        uint32_t motor_idx = 0;
        if (mode_ != CtrlMode::BROADCAST_CURRENT) {
            if (rx_header->Identifier != packet_id_) {
                return;
            }
            motor_idx = 0;
            // if this is the state querying response packet
            if (const uint8_t packet_type = rx_data[0]; packet_type == 0x9A) {
                if (const uint8_t motor_state = rx_data[6]; motor_state == 0x00) {
                    state_.set(State::ENABLED);
                } else {
                    state_.set(State::DISABLED);
                }
                return;
            }
        } else {
            if (rx_header->Identifier < 0x141 || rx_header->Identifier > 0x144) {
                return;
            }
            motor_idx = rx_header->Identifier - 0x140 - 1;
        }

        motor_[motor_idx].report.last_receive_time.set_current();
        motor_[motor_idx].report.temperature.set(little_endian::read_uint8(rx_data, &index));
        motor_[motor_idx].report.current.set(little_endian::read_int16(rx_data, &index));
        motor_[motor_idx].report.speed.set(little_endian::read_int16(rx_data, &index));
        motor_[motor_idx].report.ecd.set(little_endian::read_uint16(rx_data, &index));
    }

    void LK_MOTOR::run_task() {
        if (mode_ != CtrlMode::BROADCAST_CURRENT) {
            // if no any packet received mark as offline and disabled
            if (!motor_[0].is_online()) {
                state_.set(State::DISABLED);
                // try to get response then we can find when it backs online
                generate_disable_packet();
                send_packet();
                return;
            }

            // below this line the motor is confirmed online

            // when the query packet received, the state will change
            // then it will skip this if statement
            if (state_.get() == State::QUERYING_STATE) {
                generate_state_check_packet();
                send_packet();
                return;
            }

            // if the command wants the motor to be enabled
            if (motor_[0].command.is_enable.get()) {
                // if it is not enabled
                if (state_.get() != State::ENABLED) {
                    // send enable packet
                    generate_enable_packet();
                    send_packet();
                    state_.set(State::QUERYING_STATE);
                    return;
                }

                // below this line the motor is confirmed enabled
                memset(shared_tx_buf_, 0, 8);
                int index = 0;
                switch (mode_) {
                    case CtrlMode::OPEN_LOOP_CURRENT: {
                        little_endian::write_uint8(0xA0, shared_tx_buf_, &index);
                        index = 4;
                        little_endian::write_int16(motor_[0].command.cmd_int16.get(), shared_tx_buf_, &index);
                        break;
                    }
                    case CtrlMode::TORQUE: {
                        little_endian::write_uint8(0xA1, shared_tx_buf_, &index);
                        index = 4;
                        little_endian::write_int16(motor_[0].command.cmd_int16.get(), shared_tx_buf_, &index);
                        break;
                    }
                    case CtrlMode::SPEED_WITH_TORQUE_LIMIT: {
                        little_endian::write_uint8(0xA2, shared_tx_buf_, &index);
                        index = 2;
                        little_endian::write_int16(motor_[0].command.cmd_int16.get(), shared_tx_buf_, &index);
                        little_endian::write_int32(motor_[0].command.cmd_int32.get(), shared_tx_buf_, &index);
                        break;
                    }
                    case CtrlMode::MULTI_ROUND_POSITION: {
                        little_endian::write_uint8(0xA3, shared_tx_buf_, &index);
                        index = 4;
                        little_endian::write_int32(motor_[0].command.cmd_int32.get(), shared_tx_buf_, &index);
                        break;
                    }
                    case CtrlMode::MULTI_ROUND_POSITION_WITH_SPEED_LIMIT: {
                        little_endian::write_uint8(0xA4, shared_tx_buf_, &index);
                        index = 2;
                        little_endian::write_uint16(motor_[0].command.cmd_uint16.get(), shared_tx_buf_, &index);
                        little_endian::write_int32(motor_[0].command.cmd_int32.get(), shared_tx_buf_, &index);
                        break;
                    }
                    case CtrlMode::SINGLE_ROUND_POSITION: {
                        little_endian::write_uint8(0xA5, shared_tx_buf_, &index);
                        little_endian::write_uint8(motor_[0].command.cmd_uint8.get(), shared_tx_buf_, &index);
                        index = 4;
                        little_endian::write_uint32(motor_[0].command.cmd_uint32.get(), shared_tx_buf_, &index);
                        break;
                    }
                    case CtrlMode::SINGLE_ROUND_POSITION_WITH_SPEED_LIMIT: {
                        little_endian::write_uint8(0xA6, shared_tx_buf_, &index);
                        little_endian::write_uint8(motor_[0].command.cmd_uint8.get(), shared_tx_buf_, &index);
                        little_endian::write_uint16(motor_[0].command.cmd_uint16.get(), shared_tx_buf_, &index);
                        little_endian::write_uint32(motor_[0].command.cmd_uint32.get(), shared_tx_buf_, &index);
                        break;
                    }
                    default: {
                        generate_disable_packet();
                        state_.set(State::DISABLED);
                    }
                }
            } else {
                // if the command wants the motor to be disabled
                // no need to query because we will continuously send disable packet
                // when we change the command to enable it will send required to send enable packet
                generate_disable_packet();
                state_.set(State::DISABLED);
            }
        } else {
            int index = 0;
            for (auto &motor : motor_) {
                little_endian::write_int16(motor.command.cmd_int16.get(), shared_tx_buf_, &index);
            }
        }

        send_packet();
    }
}
