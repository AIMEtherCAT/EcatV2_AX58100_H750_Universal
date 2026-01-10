//
// Created by Hang XU on 22/10/2025.
//
#include "buffer_utils.hpp"
#include "peripheral_utils.hpp"
#include "task_defs.hpp"

extern "C" {
#include "fdcan.h"
}


namespace aim::ecat::task::pmu_uavcan {
    PMU_UAVCAN::PMU_UAVCAN(buffer::Buffer */* buffer */) : CanRunnable(true) {
        init_peripheral(peripheral::Type::PERIPHERAL_CAN);

        period = STATE_BROADCAST_PERIOD;
        can_id_type_ = FDCAN_EXTENDED_ID;
        can_inst_ = &hfdcan2;

        shared_tx_header_.Identifier = PACKET_ID;
        shared_tx_header_.IdType = FDCAN_EXTENDED_ID;
        shared_tx_header_.TxFrameType = FDCAN_DATA_FRAME;
        shared_tx_header_.DataLength = 8;
        shared_tx_header_.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        shared_tx_header_.BitRateSwitch = FDCAN_BRS_OFF;
        shared_tx_header_.FDFormat = FDCAN_FD_CAN;
        shared_tx_header_.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        shared_tx_header_.MessageMarker = 0;
    }

    void PMU_UAVCAN::run_task() {
        uptime_.increment();
        int index = 0;
        little_endian::write_uint32(uptime_.get(), shared_tx_buf_, &index);
        // ReSharper disable once CppRedundantParentheses
        shared_tx_buf_[7] = 0xE0 | (transfer_id_.get() & 0x1F);
        transfer_id_.set((transfer_id_.get() + 1) % 32);
        send_packet();
    }

    void PMU_UAVCAN::can_recv(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data) {
        if (rx_header->DataLength < 1 || rx_header->DataLength > 8) {
            return;
        }
        parse_tail_byte(rx_data[rx_header->DataLength - 1]);

        const uint8_t is_first = tail_.start;
        const uint8_t is_last = tail_.end;
        if (
            !rx_state_.initialized ||
            HAL_GetTick() - last_receive_time_.get() > TID_TIMEOUT ||
            (
                is_first &&
                static_cast<uint8_t>((tail_.tid - rx_state_.transfer_id) & 0x1F) > 1
            )) {
            rx_state_.initialized = 1;
            rx_state_.transfer_id = tail_.tid;
            rx_state_.toggle = 0;
            rx_state_.len = 0;
            rx_state_.crc = 0xFFFF;

            if (!is_first) {
                rx_state_.transfer_id = (tail_.tid + 1) & 0x1F;
                return;
            }
        }

        if (tail_.tid != rx_state_.transfer_id) {
            return;
        }

        if (tail_.toggle != rx_state_.toggle) {
            return;
        }

        last_receive_time_.set_current();
        rx_state_.toggle ^= 1;

        uint8_t data_length = rx_header->DataLength - 1;
        int index = 0;

        if (is_first) {
            if (data_length < 2) {
                return;
            }
            rx_state_.crc = rx_data[index] | rx_data[index + 1] << 8;
            data_length -= 2;
            index += 2;
        }

        if (rx_state_.len + data_length > BUF_SIZE) {
            rx_state_.len = 0;
            return;
        }

        for (int i = 0; i < data_length; i++) {
            rx_state_.buffer[rx_state_.len++] = rx_data[index + i];
        }

        if (is_last) {
            if (rx_state_.buffer[0] != 0) {
                recv_buf_.write(rx_state_.buffer, 6);
            }

            rx_state_.len = 0;
            rx_state_.toggle = 0;
            rx_state_.transfer_id = (rx_state_.transfer_id + 1) & 0x1F;
            memset(rx_state_.buffer, 0, BUF_SIZE);
        }
    }

    void PMU_UAVCAN::write_to_master(buffer::Buffer *slave_to_master_buf) {
        uint8_t recv_buf[6] = {};
        recv_buf_.read(recv_buf, 6);
        slave_to_master_buf->write(recv_buf, 6);
    }
}