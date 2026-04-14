//
// Created by hang on 2026/4/12.
//
#include "buffer_utils.hpp"
#include "peripheral_utils.hpp"
#include "task_defs.hpp"

extern "C" {
#include "fdcan.h"
}

namespace aim::ecat::task::dd_motor {
    DD_MOTOR::DD_MOTOR(buffer::Buffer *buffer) : CanRunnable(true, TaskType::DD_MOTOR) {
        switch (buffer->read_uint8(buffer::EndianType::LITTLE)) {
            case 0x01: {
                init_peripheral(peripheral::Type::PERIPHERAL_CAN_1M);
                break;
            }
            case 0x02: {
                init_peripheral(peripheral::Type::PERIPHERAL_CAN_500K);
                break;
            }
            default: {
            }
        }

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
        can_cmd_packet_id_ = buffer->read_uint32(buffer::EndianType::LITTLE);

        can_id_type_ = FDCAN_STANDARD_ID;
        shared_tx_header_.Identifier = can_cmd_packet_id_;
        shared_tx_header_.IdType = FDCAN_STANDARD_ID;
        shared_tx_header_.TxFrameType = FDCAN_DATA_FRAME;
        shared_tx_header_.DataLength = 8;
        shared_tx_header_.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        shared_tx_header_.BitRateSwitch = FDCAN_BRS_OFF;
        shared_tx_header_.FDFormat = FDCAN_CLASSIC_CAN;
        shared_tx_header_.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        shared_tx_header_.MessageMarker = 0;

        for (Motor &motor: motors_) {
            // 0x96 + motor id
            motor.report_packet_id = buffer->read_uint32(buffer::EndianType::LITTLE);
            if (motor.report_packet_id != 0) {
                motor.is_exist = true;
                enabled_motors_++;
                motor.cmd_packet_idx = (motor.report_packet_id - 0x96) % 4 - 1;
            }
        }

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

        for (Motor &motor: motors_) {
            if (!motor.is_exist) {
                continue;
            }
            switch (buffer->read_uint8(buffer::EndianType::LITTLE)) {
                case 0x01: {
                    motor.mode = CtrlMode::OPEN_LOOP_VOLTAGE;
                    break;
                }
                case 0x02: {
                    motor.mode = CtrlMode::CLOSED_LOOP_CURRENT;
                    break;
                }
                case 0x03: {
                    motor.mode = CtrlMode::SPEED;
                    break;
                }
                case 0x04: {
                    motor.mode = CtrlMode::SINGLE_ROUND_POSITION;
                    break;
                }
                default: {
                }
            }
        }
    }

    void DD_MOTOR::write_to_master(buffer::Buffer *slave_to_master_buf) {
        for (Motor &motor: motors_) {
            if (!motor.is_exist) {
                continue;
            }
            slave_to_master_buf->write_uint8(buffer::EndianType::LITTLE, motor.is_online());
            slave_to_master_buf->write_uint16(buffer::EndianType::LITTLE, motor.report.ecd.get());
            slave_to_master_buf->write_int16(buffer::EndianType::LITTLE, motor.report.rpm.get());
            slave_to_master_buf->write_int16(buffer::EndianType::LITTLE, motor.report.current.get());
            slave_to_master_buf->write_uint8(buffer::EndianType::LITTLE, motor.report.mode.get());
            slave_to_master_buf->write_uint8(buffer::EndianType::LITTLE, motor.report.error.get());
        }
    }

    void DD_MOTOR::read_from_master(buffer::Buffer *master_to_slave_buf) {
        for (Motor &motor: motors_) {
            if (!motor.is_exist) {
                continue;
            }
            motor.command.is_enable.set(master_to_slave_buf->read_uint8(buffer::EndianType::LITTLE));
            motor.command.cmd.set(master_to_slave_buf->read_int16(buffer::EndianType::LITTLE));
        }
    }

    void DD_MOTOR::on_connection_lost() {
        if (connection_lost_action_ == ConnectionLostAction::RESET_TO_DEFAULT) {
            for (Motor &motor: motors_) {
                if (!motor.is_exist) {
                    continue;
                }
                motor.command.is_enable.set(0);
                motor.command.cmd.set(0);
            }
        }
    }

    void DD_MOTOR::can_recv(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data) {
        for (Motor &motor: motors_) {
            if (motor.state == State::SETTING_FEEDBACK) {
                motor.state = State::ONLINE;
                continue;
            }

            if (rx_header->Identifier != motor.report_packet_id) {
                continue;
            }
            int index = 0;
            motor.report.rpm.set(big_endian::read_int16(rx_data, &index));
            motor.report.current.set(big_endian::read_int16(rx_data, &index));
            motor.report.ecd.set(big_endian::read_uint16(rx_data, &index));
            motor.report.error.set(big_endian::read_uint8(rx_data, &index));
            motor.report.mode.set(big_endian::read_uint8(rx_data, &index));
            motor.report.last_receive_time.set_current();

            if (motor.state == State::OFFLINE) {
                motor.state = State::SETTING_FEEDBACK;
            }
            return;
        }
    }

    void DD_MOTOR::run_task() {
        memset(cmds_, 0, 8);
        uint8_t send_mode_change_packet = 0;
        uint8_t send_feedback_change_packet = 0;

        for (Motor &motor: motors_) {
            if (!motor.is_exist) {
                continue;
            }
            // if disconnected switch to offline state
            if (!motor.is_online()) {
                motor.state = State::OFFLINE;
            }

            // reset motor mode type
            // minus 1 = idx e.g. 0x97 = id 1, 0x97 - 0x96 - 1 = 0 = idx 0
            mode_change_packet[motor.report_packet_id - 0x96 - 1] = static_cast<uint8_t>(motor.mode);
            // let feedback freq eq to control freq
            feedback_mode_change_packet[motor.report_packet_id - 0x96 - 1] = static_cast<uint8_t>(period) & 0x7F;

            switch (motor.state) {
                case State::OFFLINE: {
                    // offline state trigger disable cmd
                    send_mode_change_packet++;
                    // override to disable state
                    mode_change_packet[motor.report_packet_id - 0x96 - 1] = 0x09;
                    break;
                }
                case State::SETTING_FEEDBACK: {
                    send_feedback_change_packet++;
                    break;
                }
                case State::ONLINE: {
                    if (motor.command.is_enable.get()) {
                        if (motor.report.mode.get() == 0x09) {
                            // if disabled, req online
                            send_mode_change_packet++;
                        } else {
                            cmds_[motor.cmd_packet_idx] = motor.command.cmd.get();
                        }
                    } else {
                        if (motor.report.mode.get() != 0x09) {
                            send_mode_change_packet++;
                            // override to disable state
                            mode_change_packet[motor.report_packet_id - 0x96 - 1] = 0x09;
                        }
                    }
                    break;
                }

                default: {
                }
            }
        }

        if (send_mode_change_packet) {
            memcpy(shared_tx_buf_, mode_change_packet, 8);
            shared_tx_header_.Identifier = 0x105;
            send_packet();
        }

        if (send_feedback_change_packet) {
            memcpy(shared_tx_buf_, feedback_mode_change_packet, 8);
            shared_tx_header_.Identifier = 0x106;
            send_packet();
        }

        if (!(send_mode_change_packet == enabled_motors_ || send_feedback_change_packet == enabled_motors_)) {
            int index = 0;
            for (const int16_t cmd: cmds_) {
                big_endian::write_int16(cmd, shared_tx_buf_, &index);
            }
            shared_tx_header_.Identifier = can_cmd_packet_id_;
            send_packet();
        }
    }
}
