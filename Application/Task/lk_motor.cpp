//
// Created by Hang XU on 22/10/2025.
//
#include "buffer_manager.hpp"
#include "peripheral_manager.hpp"
#include "task_defs.hpp"

namespace aim::ecat::task::lk_motor {
    LK_MOTOR::LK_MOTOR(buffer::Buffer *buffer) : CanRunnable(true) {
        init_peripheral(peripheral::Type::PERIPHERAL_CAN);

        period = buffer->read_uint16();

        packet_id_ = buffer->read_uint32();
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

        switch (buffer->read_uint8()) {
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

        switch (buffer->read_uint8()) {
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
            default: {
            }
        }
    }

    void LK_MOTOR::write_to_master(buffer::Buffer *slave_to_master_buf) {
        slave_to_master_buf->write_uint8(is_online());
        slave_to_master_buf->write_int16(report_.current.get());
        slave_to_master_buf->write_int16(report_.speed.get());
        slave_to_master_buf->write_uint16(report_.ecd.get());
        slave_to_master_buf->write_uint8(report_.temperature.get());
    }

    void LK_MOTOR::read_from_master(buffer::Buffer *master_to_slave_buf) {
        command_.is_enable.set(master_to_slave_buf->read_uint8());

        switch (mode_) {
            case CtrlMode::OPEN_LOOP_CURRENT:
            case CtrlMode::TORQUE: {
                command_.cmd_int16.set(master_to_slave_buf->read_int16());
                break;
            }
            case CtrlMode::SPEED_WITH_TORQUE_LIMIT: {
                command_.cmd_int16.set(master_to_slave_buf->read_int16());
                command_.cmd_int32.set(master_to_slave_buf->read_int32());
                break;
            }
            case CtrlMode::MULTI_ROUND_POSITION: {
                command_.cmd_int32.set(master_to_slave_buf->read_int32());
                break;
            }
            case CtrlMode::MULTI_ROUND_POSITION_WITH_SPEED_LIMIT: {
                command_.cmd_uint16.set(master_to_slave_buf->read_uint16());
                command_.cmd_int32.set(master_to_slave_buf->read_int32());
                break;
            }
            case CtrlMode::SINGLE_ROUND_POSITION: {
                command_.cmd_uint8.set(master_to_slave_buf->read_uint8());
                command_.cmd_uint32.set(master_to_slave_buf->read_uint32());
                break;
            }
            case CtrlMode::SINGLE_ROUND_POSITION_WITH_SPEED_LIMIT: {
                command_.cmd_uint8.set(master_to_slave_buf->read_uint8());
                command_.cmd_uint16.set(master_to_slave_buf->read_uint16());
                command_.cmd_uint32.set(master_to_slave_buf->read_uint32());
                break;
            }
            default: {
            }
        }
    }

    void LK_MOTOR::can_recv(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data) {
        if (rx_header->Identifier != packet_id_) {
            return;
        }
        report_.last_receive_time.set_current();

        int index = 1;
        // if this is the state querying response packet
        if (const uint8_t packet_type = rx_data[0]; packet_type == 0x9A) {
            if (const uint8_t motor_state = rx_data[6]; motor_state == 0x00) {
                state_.set(State::ENABLED);
            } else {
                state_.set(State::DISABLED);
            }
        } else {
            report_.temperature.set(little_endian::read_uint8(rx_data, &index));
            report_.current.set(little_endian::read_int16(rx_data, &index));
            report_.speed.set(little_endian::read_int16(rx_data, &index));
            report_.ecd.set(little_endian::read_uint16(rx_data, &index));
        }
    }

    void LK_MOTOR::run_task() {
        // if no any packet received mark as offline and disabled
        if (!is_online()) {
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
        if (command_.is_enable.get()) {
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
                    little_endian::write_int16(command_.cmd_int16.get(), shared_tx_buf_, &index);
                    break;
                }
                case CtrlMode::TORQUE: {
                    little_endian::write_uint8(0xA1, shared_tx_buf_, &index);
                    index = 4;
                    little_endian::write_int16(command_.cmd_int16.get(), shared_tx_buf_, &index);
                    break;
                }
                case CtrlMode::SPEED_WITH_TORQUE_LIMIT: {
                    little_endian::write_uint8(0xA2, shared_tx_buf_, &index);
                    index = 2;
                    little_endian::write_int16(command_.cmd_int16.get(), shared_tx_buf_, &index);
                    little_endian::write_int32(command_.cmd_int32.get(), shared_tx_buf_, &index);
                    break;
                }
                case CtrlMode::MULTI_ROUND_POSITION: {
                    little_endian::write_uint8(0xA3, shared_tx_buf_, &index);
                    index = 4;
                    little_endian::write_int32(command_.cmd_int32.get(), shared_tx_buf_, &index);
                    break;
                }
                case CtrlMode::MULTI_ROUND_POSITION_WITH_SPEED_LIMIT: {
                    little_endian::write_uint8(0xA4, shared_tx_buf_, &index);
                    index = 2;
                    little_endian::write_uint16(command_.cmd_uint16.get(), shared_tx_buf_, &index);
                    little_endian::write_int32(command_.cmd_int32.get(), shared_tx_buf_, &index);
                    break;
                }
                case CtrlMode::SINGLE_ROUND_POSITION: {
                    little_endian::write_uint8(0xA5, shared_tx_buf_, &index);
                    little_endian::write_uint8(command_.cmd_uint8.get(), shared_tx_buf_, &index);
                    index = 4;
                    little_endian::write_uint32(command_.cmd_uint32.get(), shared_tx_buf_, &index);
                    break;
                }
                case CtrlMode::SINGLE_ROUND_POSITION_WITH_SPEED_LIMIT: {
                    little_endian::write_uint8(0xA6, shared_tx_buf_, &index);
                    little_endian::write_uint8(command_.cmd_uint8.get(), shared_tx_buf_, &index);
                    little_endian::write_uint16(command_.cmd_uint16.get(), shared_tx_buf_, &index);
                    little_endian::write_uint32(command_.cmd_uint32.get(), shared_tx_buf_, &index);
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
        send_packet();
    }
}