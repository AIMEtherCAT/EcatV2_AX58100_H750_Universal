//
// Created by Hang XU on 22/10/2025.
//
#include "buffer_manager.hpp"
#include "peripheral_manager.hpp"
#include "task_defs.hpp"

namespace aim::ecat::task::hipnuc_imu {
    HIPNUC_IMU_CAN::HIPNUC_IMU_CAN(buffer::Buffer */* buffer */) : CanRunnable(false) {
        init_peripheral(peripheral::Type::PERIPHERAL_CAN);

        can_id_type_ = FDCAN_STANDARD_ID;
        can_inst_ = &hfdcan2;
    }

    void HIPNUC_IMU_CAN::can_recv(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data) {
        uint8_t current_buf[21] = {};
        buf_.read(current_buf, 21);
        switch (rx_header->Identifier) {
            case 0x01: {
                memcpy(current_buf, rx_data, 8);
                break;
            }
            case 0x02: {
                memcpy(current_buf + 8, rx_data, 8);
                break;
            }
            case 0x03: {
                memcpy(current_buf + 8 + 8, rx_data, 5);
                break;
            }
            default: {
            }
        }
        buf_.write(current_buf, 21);
    }

    void HIPNUC_IMU_CAN::write_to_master(buffer::Buffer *slave_to_master_buf) {
        uint8_t current_buf[21] = {};
        buf_.read(current_buf, 21);
        slave_to_master_buf->write(current_buf, 21);
    }
}
