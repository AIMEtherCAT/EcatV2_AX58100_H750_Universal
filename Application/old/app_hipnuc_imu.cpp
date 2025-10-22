//
// Created by Hang XU on 04/06/2025.
//
#include "../task_defs.h"
#include <cstring>
#include "io_utils.h"
#include "dshot.h"

extern "C" {
#include "fdcan.h"
#include "peripheral_conf.hpp"
}

App_HIPNUC_IMU::App_HIPNUC_IMU(uint8_t *args, int *offset) {
    this->can_inst = &hfdcan2;
    this->can_id_type = FDCAN_STANDARD_ID;
    init_can();
    memset(imu_buf, 0, sizeof(imu_buf));
}

void App_HIPNUC_IMU::collect_outputs(uint8_t *output, int *output_offset) {
    memcpy(output + *(output_offset), imu_buf, (8 + 8 + 5));
    (*output_offset) += (8 + 8 + 5);
}

void App_HIPNUC_IMU::can_recv(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data) {
    switch (rx_header->Identifier) {
        case 0x01: {
            memcpy(imu_buf, rx_data, 8);
            break;
        }
        case 0x02: {
            memcpy(imu_buf + 8, rx_data, 8);
            break;
        }
        case 0x03: {
            memcpy(imu_buf + 8 + 8, rx_data, 5);
            break;
        }
        default: {
        }
    }
}

void App_HIPNUC_IMU::exit() {
    deinit_can();
}
