//
// Created by Hang XU on 22/10/2025.
//
#include "buffer_manager.hpp"
#include "peripheral_manager.hpp"
#include "task_defs.hpp"

namespace aim::ecat::task::dm_motor {
    DM_MOTOR::DM_MOTOR(buffer::Buffer *buffer) {
        init_peripheral(peripheral::Type::PERIPHERAL_CAN);

        period = buffer->read_uint16();

        can_id_ = buffer->read_uint16();
        master_id_ = buffer->read_uint16();
        can_id_type_ = FDCAN_STANDARD_ID;
        shared_tx_header_.IdType = FDCAN_STANDARD_ID;
        shared_tx_header_.TxFrameType = FDCAN_DATA_FRAME;
        shared_tx_header_.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        shared_tx_header_.BitRateSwitch = FDCAN_BRS_OFF;
        shared_tx_header_.FDFormat = FDCAN_CLASSIC_CAN;
        shared_tx_header_.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        shared_tx_header_.MessageMarker = 0;

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
                mode_ = CtrlMode::MIT;
                break;
            }
            case 0x02: {
                mode_ = CtrlMode::POSITION_WITH_SPEED_LIMIT;
                break;
            }
            case 0x03: {
                mode_ = CtrlMode::SPEED;
                break;
            }
            default: {
            }
        }
        shared_tx_header_.Identifier = get_control_packet_id();
        mode_change_buf_[0] = can_id_ & 0x0F;
        mode_change_buf_[1] = (can_id_ >> 4) & 0x0F;
        mode_change_buf_[2] = 0x55;
        mode_change_buf_[3] = 0x0A;
        int index = 4;
        little_endian::write_uint32(static_cast<uint32_t>(mode_), mode_change_buf_, &index);

        change_state(State::OFFLINE);
    }

    void DM_MOTOR::write_to_master(buffer::Buffer *slave_to_master_buf) {
        uint8_t report_buf[8] = {};
        report_.read(report_buf, 8);
        slave_to_master_buf->write(report_buf, 8);
        slave_to_master_buf->write_uint8(is_online());
    }

    void DM_MOTOR::read_from_master(buffer::Buffer *master_to_slave_buf) {
        cmd_.is_enable.set(master_to_slave_buf->read_uint8());
        uint8_t cmd_buf[8] = {};
        switch (mode_) {
            case CtrlMode::MIT:
            case CtrlMode::POSITION_WITH_SPEED_LIMIT: {
                master_to_slave_buf->read(cmd_buf, 8);
                break;
            }
            case CtrlMode::SPEED: {
                master_to_slave_buf->read(cmd_buf, 4);
                break;
            }
            default: {
                break;
            }
        }
        cmd_.cmd.write(cmd_buf, 8);
    }

    void DM_MOTOR::can_recv(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data) {
        if (rx_header->Identifier != master_id_) {
            return;
        }
        last_receive_time.set_current();

        if (memcmp(rx_data, mode_change_buf_, 8) == 0) {
            change_state(State::MODE_CHANGED);
            return;
        }

        uint8_t report_buf[8] = {};
        memcpy(report_buf, rx_data, 8);
        report_.write(report_buf, 8);

        if (report_buf[0] >> 4 == 1) {
            state_.is_motor_enabled.set();
        } else {
            state_.is_motor_enabled.clear();
        }
    }


    void DM_MOTOR::run_task() {
        // check if received any packet from the motor
        if (!is_online()) {
            state_.is_motor_enabled.clear();
            change_state(State::OFFLINE);
        }

        switch (get_current_state()) {
            case State::OFFLINE: {
                // set the mode once a motor reconnected
                // cuz the motor may be restarted or changed
                if (is_online()) {
                    change_state(State::SENDING_MODE_CHANGE);
                    return;
                }

                // keeping sending dummy packet to get reply frame from the motor
                generate_disable_packet();
                shared_tx_header_.DataLength = 8;
                send_packet();
                break;
            }
            case State::SENDING_MODE_CHANGE: {
                // send mode change packet, only supported V3 hw/sw
                generate_mode_change_packet();
                shared_tx_header_.DataLength = 8;
                send_packet();
                change_state(State::PENDING_MODE_CHANGE);
                break;
            }
            case State::PENDING_MODE_CHANGE: {
                // if motor online but not received any mode-change confirmation
                // it means this motor is V2 hw/sw, which don't support mode change packet
                // in this case just skip the mode change state
                if (HAL_GetTick() - get_state_enter_time() > MODE_CHANGE_BYPASS_TIMEOUT) {
                    change_state(State::MODE_CHANGE_BYPASSED);
                    return;
                }

                // keeping sending request packets
                generate_mode_change_packet();
                shared_tx_header_.DataLength = 8;
                send_packet();
                break;
            }
            case State::MODE_CHANGED:
            case State::MODE_CHANGE_BYPASSED: {
                // below this line the motor is confirmed online and ready to control

                if (cmd_.is_enable.get()) {
                    // motor not enabled
                    if (!state_.is_motor_enabled.get()) {
                        generate_enable_packet();
                        shared_tx_header_.DataLength = 8;
                    } else {
                        uint8_t cmd_buf[8] = {};
                        cmd_.cmd.read(cmd_buf, 8);

                        switch (mode_) {
                            case CtrlMode::MIT:
                            case CtrlMode::POSITION_WITH_SPEED_LIMIT: {
                                memcpy(shared_tx_buf_, cmd_buf, 8);
                                shared_tx_header_.DataLength = 8;
                                break;
                            }
                            case CtrlMode::SPEED: {
                                memcpy(shared_tx_buf_, cmd_buf, 4);
                                shared_tx_header_.DataLength = 4;
                                break;
                            }
                            default: {
                                state_.is_motor_enabled.clear();
                                generate_disable_packet();
                                shared_tx_header_.DataLength = 8;
                                break;
                            }
                        }
                    }
                } else {
                    state_.is_motor_enabled.clear();
                    generate_disable_packet();
                    shared_tx_header_.DataLength = 8;
                }

                send_packet();
                break;
            }
            default: {
                break;
            }
        }
    }
}
