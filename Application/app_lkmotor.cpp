//
// Created by Hang XU on 03/08/2025.
//
#include "task_defs.h"
#include <cstring>
#include "IOUtils.h"

extern "C" {
#include "fdcan.h"
#include "device_conf.h"
}

App_LkMotor::App_LkMotor(uint8_t *args, int *offset) {
    this->running = 1;
    this->period = read_uint16(args, offset);

    this->can_packet_id = read_uint32(args, offset);
    this->can_inst_id = read_uint8(args, offset);

    switch (this->can_inst_id) {
        case 0x01: {
            this->can_inst = &hfdcan1;
            break;
        }
        case 0x02: {
            this->can_inst = &hfdcan2;
        }
        default: {
        }
    }

    switch (read_uint8(args, offset)) {
        case 0x01: {
            this->ctrl_mode = OPENLOOP_CURRENT_LK;
            break;
        }
        case 0x02: {
            this->ctrl_mode = TORQUE_LK;
            break;
        }
        case 0x03: {
            this->ctrl_mode = SPEED_WITH_TORQUE_LIMIT_LK;
            break;
        }
        case 0x04: {
            this->ctrl_mode = MULTI_ROUND_POSITION_LK;
            break;
        }
        case 0x05: {
            this->ctrl_mode = MULTI_ROUND_POSITION_WITH_SPEED_LIMIT_LK;
            break;
        }
        case 0x06: {
            this->ctrl_mode = SINGLE_ROUND_POSITION_LK;
            break;
        }
        case 0x07: {
            this->ctrl_mode = SINGLE_ROUND_POSITION_WITH_SPEED_LIMIT_LK;
            break;
        }
        default: {
        }
    }

    this->can_id_type = FDCAN_STANDARD_ID;
    this->tx_header.IdType = FDCAN_STANDARD_ID;
    this->tx_header.TxFrameType = FDCAN_DATA_FRAME;
    this->tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    this->tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    this->tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    this->tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    this->tx_header.MessageMarker = 0;
    this->tx_header.DataLength = 8;
    this->tx_header.Identifier = this->can_packet_id;
    init_can();

    this->last_cmd = 0;
    this->disable_motor();
}

void App_LkMotor::collect_outputs(uint8_t *output, int *output_offset) {
    write_int16(this->motor_status.current, output, output_offset);
    write_int16(this->motor_status.rpm, output, output_offset);
    write_uint16(this->motor_status.ecd, output, output_offset);
    write_uint8(this->motor_status.temperature, output, output_offset);
}

void App_LkMotor::collect_inputs(uint8_t *input, int *input_offset) {
    this->cmd_motor_enable = read_uint8(input, input_offset);
    switch (this->ctrl_mode) {
        case OPENLOOP_CURRENT_LK: {
            this->cmd_torque = read_int16(input, input_offset);
            break;
        }
        case TORQUE_LK: {
            this->cmd_torque = read_int16(input, input_offset);
            break;
        }
        case SPEED_WITH_TORQUE_LIMIT_LK: {
            this->cmd_torque = read_int16(input, input_offset);
            this->cmd_speed = read_int32(input, input_offset);
            break;
        }
        case MULTI_ROUND_POSITION_LK: {
            this->cmd_multi_round_angle = read_int32(input, input_offset);
            break;
        }
        case MULTI_ROUND_POSITION_WITH_SPEED_LIMIT_LK: {
            this->cmd_speed_limit = read_uint16(input, input_offset);
            this->cmd_multi_round_angle = read_int32(input, input_offset);
            break;
        }
        case SINGLE_ROUND_POSITION_LK: {
            this->cmd_direction = read_uint8(input, input_offset);
            this->cmd_single_round_angle = read_uint32(input, input_offset);
            break;
        }
        case SINGLE_ROUND_POSITION_WITH_SPEED_LIMIT_LK: {
            this->cmd_direction = read_uint8(input, input_offset);
            this->cmd_speed_limit = read_uint16(input, input_offset);
            this->cmd_single_round_angle = read_uint32(input, input_offset);
            break;
        }
        default: {
        }
    }
}

void App_LkMotor::run_task() {
    if (!running) {
        return;
    }

    if (last_cmd != cmd_motor_enable) {
        if (this->cmd_motor_enable == 1) {
            this->enable_motor();
        } else {
            this->disable_motor();
        }
        state_change_confirm++;
        if (state_change_confirm >= 3) {
            last_cmd = cmd_motor_enable;
            state_change_confirm = 0;
        }
        return;
    }
    memset(tx_data, 0, sizeof(tx_data));

    switch (this->ctrl_mode) {
        case OPENLOOP_CURRENT_LK: {
            tx_data[0] = 0xA0;

            tx_data[4] = *(uint8_t *) (&cmd_torque);
            tx_data[5] = *((uint8_t *) (&cmd_torque) + 1);
            break;
        }
        case TORQUE_LK: {
            tx_data[0] = 0xA1;

            tx_data[4] = *(uint8_t *) (&cmd_torque);
            tx_data[5] = *((uint8_t *) (&cmd_torque) + 1);
            break;
        }
        case SPEED_WITH_TORQUE_LIMIT_LK: {
            tx_data[0] = 0xA2;

            tx_data[2] = *(uint8_t *) (&cmd_torque);
            tx_data[3] = *((uint8_t *) (&cmd_torque) + 1);

            tx_data[4] = *(uint8_t *) (&cmd_speed);
            tx_data[5] = *((uint8_t *) (&cmd_speed) + 1);
            tx_data[6] = *((uint8_t *) (&cmd_speed) + 2);
            tx_data[7] = *((uint8_t *) (&cmd_speed) + 3);
            break;
        }
        case MULTI_ROUND_POSITION_LK: {
            tx_data[0] = 0xA3;

            tx_data[4] = *(uint8_t *) (&cmd_multi_round_angle);
            tx_data[5] = *((uint8_t *) (&cmd_multi_round_angle) + 1);
            tx_data[6] = *((uint8_t *) (&cmd_multi_round_angle) + 2);
            tx_data[7] = *((uint8_t *) (&cmd_multi_round_angle) + 3);
            break;
        }
        case MULTI_ROUND_POSITION_WITH_SPEED_LIMIT_LK: {
            tx_data[0] = 0xA4;

            tx_data[2] = *(uint8_t *) (&cmd_speed_limit);
            tx_data[3] = *((uint8_t *) (&cmd_speed_limit) + 1);

            tx_data[4] = *(uint8_t *) (&cmd_multi_round_angle);
            tx_data[5] = *((uint8_t *) (&cmd_multi_round_angle) + 1);
            tx_data[6] = *((uint8_t *) (&cmd_multi_round_angle) + 2);
            tx_data[7] = *((uint8_t *) (&cmd_multi_round_angle) + 3);
            break;
        }
        case SINGLE_ROUND_POSITION_LK: {
            tx_data[0] = 0xA5;

            tx_data[0] = cmd_direction;

            tx_data[4] = *(uint8_t *) (&cmd_single_round_angle);
            tx_data[5] = *((uint8_t *) (&cmd_single_round_angle) + 1);
            tx_data[6] = *((uint8_t *) (&cmd_single_round_angle) + 2);
            tx_data[7] = *((uint8_t *) (&cmd_single_round_angle) + 3);
            break;
        }
        case SINGLE_ROUND_POSITION_WITH_SPEED_LIMIT_LK: {
            tx_data[0] = 0xA6;

            tx_data[0] = cmd_direction;

            tx_data[2] = *(uint8_t *) (&cmd_speed_limit);
            tx_data[3] = *((uint8_t *) (&cmd_speed_limit) + 1);

            tx_data[4] = *(uint8_t *) (&cmd_single_round_angle);
            tx_data[5] = *((uint8_t *) (&cmd_single_round_angle) + 1);
            tx_data[6] = *((uint8_t *) (&cmd_single_round_angle) + 2);
            tx_data[7] = *((uint8_t *) (&cmd_single_round_angle) + 3);
            break;
        }
        default: {
        }
    }

    HAL_FDCAN_AddMessageToTxFifoQ(this->can_inst, &this->tx_header, this->tx_data);
}

void App_LkMotor::can_recv(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data) {
    if (this->can_packet_id == rx_header->Identifier) {
        this->motor_status.temperature = rx_data[1];

        int16_t cur;
        *(uint8_t *) (&cur) = rx_data[2];
        *((uint8_t *) (&cur) + 1) = rx_data[3];
        this->motor_status.current = cur;

        int16_t spd;
        *(uint8_t *) (&spd) = rx_data[4];
        *((uint8_t *) (&spd) + 1) = rx_data[5];
        this->motor_status.rpm = ((float) spd) * 60.0f / 360.0f;

        *(uint8_t *) (&(this->motor_status.ecd)) = rx_data[6];
        *((uint8_t *) (&(this->motor_status.ecd)) + 1) = rx_data[7];
    }
}

void App_LkMotor::exit() {
    deinit_can();
}

void App_LkMotor::enable_motor() {
    memset(tx_data, 0, sizeof(tx_data));
    tx_data[0] = 0x88;
    HAL_FDCAN_AddMessageToTxFifoQ(this->can_inst, &this->tx_header, this->tx_data);
}

void App_LkMotor::disable_motor() {
    memset(tx_data, 0, sizeof(tx_data));
    tx_data[0] = 0x80;
    HAL_FDCAN_AddMessageToTxFifoQ(this->can_inst, &this->tx_header, this->tx_data);
}

