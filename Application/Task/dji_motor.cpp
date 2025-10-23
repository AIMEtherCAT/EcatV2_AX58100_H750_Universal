//
// Created by Hang XU on 22/10/2025.
//
#include "buffer_manager.hpp"
#include "peripheral_manager.hpp"
#include "task_defs.hpp"

namespace aim::ecat::task::dji_motor {
    DJI_MOTOR::DJI_MOTOR(buffer::Buffer *buffer) {
        init_peripheral(peripheral::Type::PERIPHERAL_CAN);

        period = buffer->read_uint16();

        can_id_type_ = FDCAN_STANDARD_ID;
        shared_tx_header_.Identifier = buffer->read_uint32();
        shared_tx_header_.IdType = FDCAN_STANDARD_ID;
        shared_tx_header_.TxFrameType = FDCAN_DATA_FRAME;
        shared_tx_header_.DataLength = 8;
        shared_tx_header_.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        shared_tx_header_.BitRateSwitch = FDCAN_BRS_OFF;
        shared_tx_header_.FDFormat = FDCAN_CLASSIC_CAN;
        shared_tx_header_.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        shared_tx_header_.MessageMarker = 0;

        for (Motor &motor: motors_) {
            motor.report_packet_id = buffer->read_uint32();
            if (motor.report_packet_id != 0) {
                motor.is_exist = true;
            }
        }

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

        for (Motor &motor: motors_) {
            if (!motor.is_exist) {
                continue;
            }
            switch (buffer->read_uint8()) {
                case 0x01: {
                    motor.mode = CtrlMode::OPEN_LOOP_CURRENT;
                    break;
                }
                case 0x02: {
                    motor.mode = CtrlMode::SPEED;
                    motor.speed_pid.basic_init(
                        buffer->read_float(),
                        buffer->read_float(),
                        buffer->read_float(),
                        buffer->read_float(),
                        buffer->read_float()
                    );
                    break;
                }
                case 0x03: {
                    motor.mode = CtrlMode::SINGLE_ROUND_POSITION;
                    motor.speed_pid.basic_init(
                        buffer->read_float(),
                        buffer->read_float(),
                        buffer->read_float(),
                        buffer->read_float(),
                        buffer->read_float()
                    );
                    motor.angle_pid.basic_init(
                        buffer->read_float(),
                        buffer->read_float(),
                        buffer->read_float(),
                        buffer->read_float(),
                        buffer->read_float()
                    );
                    break;
                }
                default: {
                }
            }
        }
    }

    void DJI_MOTOR::write_to_master(buffer::Buffer *slave_to_master_buf) {
        for (Motor &motor: motors_) {
            if (!motor.is_exist) {
                continue;
            }
            slave_to_master_buf->write_uint8(motor.is_online());
            slave_to_master_buf->write_uint16(motor.report.ecd.get());
            slave_to_master_buf->write_int16(motor.report.rpm.get());
            slave_to_master_buf->write_int16(motor.report.current.get());
            slave_to_master_buf->write_uint8(motor.report.temperature.get());
        }
    }

    void DJI_MOTOR::read_from_master(buffer::Buffer *master_to_slave_buf) {
        for (Motor &motor: motors_) {
            if (!motor.is_exist) {
                continue;
            }
            motor.command.is_enable.set(master_to_slave_buf->read_uint8());
            motor.command.cmd.set(master_to_slave_buf->read_int16());
        }
    }

    void DJI_MOTOR::can_recv(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data) {
        for (Motor &motor: motors_) {
            if (rx_header->Identifier != motor.report_packet_id) {
                continue;
            }
            int index = 0;
            motor.report.ecd.set(big_endian::read_uint16(rx_data, &index));
            motor.report.rpm.set(big_endian::read_int16(rx_data, &index));
            motor.report.current.set(big_endian::read_int16(rx_data, &index));
            motor.report.temperature.set(big_endian::read_uint8(rx_data, &index));
            motor.report.last_receive_time.set_current();
            return;
        }
    }

    void DJI_MOTOR::run_task() {
        memset(shared_tx_buf_, 0, 8);
        int index = 0;

        for (Motor &motor: motors_) {
            if (!motor.is_exist) {
                big_endian::write_uint16(0, shared_tx_buf_, &index);
                continue;
            }
            if (!motor.is_online()) {
                big_endian::write_uint16(0, shared_tx_buf_, &index);
                continue;
            }

            if (motor.command.is_enable.get()) {
                switch (motor.mode) {
                    case CtrlMode::OPEN_LOOP_CURRENT: {
                        big_endian::write_int16(motor.command.cmd.get(), shared_tx_buf_,
                                                &index);
                        break;
                    }
                    case CtrlMode::SPEED: {
                        big_endian::write_int16(
                            static_cast<int16_t>(motor.speed_pid.calculate(
                                motor.report.rpm.get(),
                                motor.command.cmd.get())),
                            shared_tx_buf_, &index);
                        break;
                    }
                    case CtrlMode::SINGLE_ROUND_POSITION: {
                        big_endian::write_int16(
                            static_cast<int16_t>(motor.speed_pid.calculate(
                                motor.report.rpm.get(),
                                -motor.speed_pid.calculate(
                                    0,
                                    calculate_err(
                                        motor.report.ecd.get(),
                                        motor.command.cmd.get())
                                )
                            )),
                            shared_tx_buf_, &index);
                        break;
                    }
                }
            } else {
                big_endian::write_uint16(0, shared_tx_buf_, &index);
            }
        }

        send_packet();
    }
}
