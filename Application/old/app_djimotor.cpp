//
// Created by Hang XU on 12/07/2025.
//
//
// Created by Hang XU on 25/05/2025.
//

#ifndef APP_DJIMOTOR_HPP
#define APP_DJIMOTOR_HPP

#include "../task_defs.h"
#include <cstring>
#include "io_utils.h"
#include "dshot.h"

extern "C" {
#include "fdcan.h"
#include "peripheral_conf.hpp"
}

App_DJIMotor::App_DJIMotor(uint8_t *args, int *offset) {
    this->running = 1;
    this->period = read_uint16(args, offset);

    this->can_packet_id = read_uint32(args, offset);
    for (int i = 0; i < 4; i++) {
        this->can_motor_report_packet_id[i] = read_uint32(args, offset);
    }
    this->can_inst_id = read_uint8(args, offset);

    switch (this->can_inst_id) {
        case 0x01: {
            this->can_inst = &hfdcan1;
            break;
        }
        case 0x02: {
            this->can_inst = &hfdcan2;
        }
    }

    for (int i = 0; i < 4; i++) {
        if (this->can_motor_report_packet_id[i] == 0) {
            continue;
        }
        switch (read_uint8(args, offset)) {
            case 0x01: {
                this->ctrl_mode[i] = OPENLOOP_CURRENT;
                break;
            }
            case 0x02: {
                this->ctrl_mode[i] = SPEED;
                PID_init_raw(&(this->speed_pid[i]),
                             read_float(args, offset),
                             read_float(args, offset),
                             read_float(args, offset),
                             read_float(args, offset),
                             read_float(args, offset));
                break;
            }
            case 0x03: {
                this->ctrl_mode[i] = SINGLE_ROUND_POSITION;
                PID_init_raw(&(this->speed_pid[i]),
                             read_float(args, offset),
                             read_float(args, offset),
                             read_float(args, offset),
                             read_float(args, offset),
                             read_float(args, offset));
                PID_init_raw(&(this->angle_pid[i]),
                             read_float(args, offset),
                             read_float(args, offset),
                             read_float(args, offset),
                             read_float(args, offset),
                             read_float(args, offset));
                break;
            }
            default: {
            }
        }
    }

    this->can_id_type = FDCAN_STANDARD_ID;
    this->shared_tx_header.IdType = FDCAN_STANDARD_ID;
    this->shared_tx_header.TxFrameType = FDCAN_DATA_FRAME;
    this->shared_tx_header.DataLength = 8;
    this->shared_tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    this->shared_tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    this->shared_tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    this->shared_tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    this->shared_tx_header.MessageMarker = 0;

    init_can();
}

void App_DJIMotor::collect_outputs(uint8_t *output, int *output_offset) {
    for (int i = 0; i < 4; i++) {
        if (this->can_motor_report_packet_id[i] == 0) {
            continue;
        }
        write_uint16(this->motor_status[i].ecd, output, output_offset);
        write_int16(this->motor_status[i].rpm, output, output_offset);
        write_int16(this->motor_status[i].current, output, output_offset);
        write_uint8(this->motor_status[i].temperature, output,
                    output_offset);
    }
}

void App_DJIMotor::collect_inputs(uint8_t *input, int *input_offset) {
    for (int i = 0; i < 4; i++) {
        if (this->can_motor_report_packet_id[i] == 0) {
            continue;
        }
        this->cmd_motor_enable[i] = read_uint8(input, input_offset);
        this->cmd[i] = read_int16(input, input_offset);
    }
}

void App_DJIMotor::run_task() {
    if (!running) {
        return;
    }

    memset(shared_tx_data, 0, sizeof(shared_tx_data));
    int16_t output[4] = {0, 0, 0, 0};

    for (int i = 0; i < 4; i++) {
        if (this->can_motor_report_packet_id[i] == 0) {
            output[i] = 0;
            continue;
        }
        if (this->cmd_motor_enable[i] == 0) {
            output[i] = 0;
            continue;
        }
        switch (this->ctrl_mode[i]) {
            case OPENLOOP_CURRENT: {
                output[i] = cmd[i];
                break;
            }
            case SPEED: {
                output[i] = PID_calc(&(this->speed_pid[i]),
                                     this->motor_status[i].rpm, cmd[i]);
                break;
            }
            case SINGLE_ROUND_POSITION: {
                output[i] = PID_calc(&(this->speed_pid[i]),
                                     this->motor_status[i].rpm,
                                     PID_calc(&(this->angle_pid[i]), 0,
                                              this->calc_err(this->motor_status[i].ecd,
                                                             cmd[i])));
                break;
            }
            default:
                ;
        }
    }
    for (int i = 0; i < 4; i++) {
        this->shared_tx_data[i * 2] = output[i] >> 8;
        this->shared_tx_data[i * 2 + 1] = output[i];
    }
    this->shared_tx_header.Identifier = this->can_packet_id;
    HAL_FDCAN_AddMessageToTxFifoQ(this->can_inst, &this->shared_tx_header, this->shared_tx_data);
}

void App_DJIMotor::can_recv(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data) {
    for (int i = 0; i < 4; i++) {
        if (this->can_motor_report_packet_id[i] == rx_header->Identifier) {
            this->motor_status[i].rpm = (int16_t) (rx_data[2] << 8
                                                   | rx_data[3]);
            this->motor_status[i].ecd = ((int16_t) (rx_data[0] << 8
                                                    | rx_data[1]));
            this->motor_status[i].current = (int16_t) (rx_data[4] << 8
                                                       | rx_data[5]);
            this->motor_status[i].temperature = rx_data[6];
        }
    }
}

void App_DJIMotor::exit() {
    deinit_can();
}

void App_DJIMotor::disable_motor() {
    memset(shared_tx_data, 0, sizeof(shared_tx_data));
    this->shared_tx_header.Identifier = this->can_packet_id;
    HAL_FDCAN_AddMessageToTxFifoQ(this->can_inst, &this->shared_tx_header, this->shared_tx_data);
}

int16_t App_DJIMotor::calc_err(int16_t current_angle, int16_t target_angle) {
    const int total_positions = 8192;
    int clockwise_difference = (target_angle - current_angle
                                + total_positions) % total_positions;
    int counterclockwise_difference = (current_angle - target_angle
                                       + total_positions) % total_positions;
    if (clockwise_difference <= counterclockwise_difference) {
        return clockwise_difference;
    } else {
        return -counterclockwise_difference;
    }
}
#endif //APP_DJIMOTOR_HPP
