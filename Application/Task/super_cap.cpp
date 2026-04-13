//
// Created by Hang XU on 2026/3/14.
//
#include "buffer_utils.hpp"
#include "peripheral_utils.hpp"
#include "task_defs.hpp"

extern "C" {
#include "fdcan.h"
}

namespace aim::ecat::task::super_cap {
    SUPER_CAP::SUPER_CAP(buffer::Buffer *buffer) : CanRunnable(true, TaskType::SUPER_CAP) {
        init_peripheral(peripheral::Type::PERIPHERAL_CAN_1M);
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
        period = 1;

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

        chassis_to_cap_packet_id_ = buffer->read_uint32(buffer::EndianType::LITTLE);
        cap_to_chassis_packet_id_ = buffer->read_uint32(buffer::EndianType::LITTLE);

        can_id_type_ = FDCAN_STANDARD_ID;
        shared_tx_header_.Identifier = chassis_to_cap_packet_id_;
        shared_tx_header_.IdType = FDCAN_STANDARD_ID;
        shared_tx_header_.TxFrameType = FDCAN_DATA_FRAME;
        shared_tx_header_.DataLength = 4;
        shared_tx_header_.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        shared_tx_header_.BitRateSwitch = FDCAN_BRS_OFF;
        shared_tx_header_.FDFormat = FDCAN_CLASSIC_CAN;
        shared_tx_header_.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        shared_tx_header_.MessageMarker = 0;
    }

    void SUPER_CAP::write_to_master(buffer::Buffer *slave_to_master_buf) {
        uint8_t current_buf[6] = {};
        rx_buf_.read(current_buf, 6);
        slave_to_master_buf->write_uint8(buffer::EndianType::LITTLE, HAL_GetTick() - last_receive_time_.get() <= 20);
        slave_to_master_buf->write(current_buf, 6);
    }

    void SUPER_CAP::read_from_master(buffer::Buffer *master_to_slave_buf) {
        uint8_t buf[4] = {};
        master_to_slave_buf->read(buf, 4);
        tx_buf_.write(buf, 4);
    }

    void SUPER_CAP::on_connection_lost() {
        if (connection_lost_action_ == ConnectionLostAction::RESET_TO_DEFAULT) {
            tx_buf_.clear();
        }
    }

    void SUPER_CAP::can_recv(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data) {
        if (rx_header->Identifier != cap_to_chassis_packet_id_) {
            return;
        }

        rx_buf_.write(rx_data, 6);
        last_receive_time_.set_current();
    }

    void SUPER_CAP::run_task() {
        tx_buf_.read(shared_tx_buf_, 4);

        send_packet();
    }
}
