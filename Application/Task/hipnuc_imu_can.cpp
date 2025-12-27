//
// Created by Hang XU on 22/10/2025.
//
#include "buffer_manager.hpp"
#include "peripheral_manager.hpp"
#include "task_defs.hpp"

namespace aim::ecat::task::hipnuc_imu {
    HIPNUC_IMU_CAN::HIPNUC_IMU_CAN(buffer::Buffer *buffer) : CanRunnable(false) {
        init_peripheral(peripheral::Type::PERIPHERAL_CAN);
        can_id_type_ = FDCAN_STANDARD_ID;

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

        packet1_id_ = buffer->read_uint32(buffer::EndianType::LITTLE);
        packet2_id_ = buffer->read_uint32(buffer::EndianType::LITTLE);
        packet3_id_ = buffer->read_uint32(buffer::EndianType::LITTLE);
    }

    void HIPNUC_IMU_CAN::can_recv(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data) {
        if (rx_header->Identifier != packet1_id_
            && rx_header->Identifier != packet2_id_
            && rx_header->Identifier != packet3_id_) {
            return;
        }

        uint8_t current_buf[21] = {};
        buf_.read(current_buf, 21);

        if (packet1_id_ == rx_header->Identifier) {
            memcpy(current_buf, rx_data, 8);
        } else if (packet2_id_ == rx_header->Identifier) {
            memcpy(current_buf + 8, rx_data, 8);
        } else if (packet3_id_ == rx_header->Identifier) { // NOLINT
            memcpy(current_buf + 8 + 8, rx_data, 5);
        }

        buf_.write(current_buf, 21);
    }

    void HIPNUC_IMU_CAN::write_to_master(buffer::Buffer *slave_to_master_buf) {
        uint8_t current_buf[21] = {};
        buf_.read(current_buf, 21);
        slave_to_master_buf->write(current_buf, 21);
    }
}