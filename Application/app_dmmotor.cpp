//
// Created by Hang XU on 02/08/2025.
//

#include "task_defs.h"
#include <cstring>
#include "IOUtils.h"

extern "C" {
#include "fdcan.h"
#include "device_conf.h"
}

App_DMMotor::App_DMMotor(uint8_t *args, int *offset) {
	this->running = 1;
	this->period = read_uint16(args, offset);

	this->ctrl_id = read_uint16(args, offset);
	this->master_id = read_uint16(args, offset);
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

	uint32_t ctrl_mode_code = 0;
	mode_change_buf[0] = ctrl_id & 0x0F;
	mode_change_buf[1] = (ctrl_id >> 4) & 0x0F;
	mode_change_buf[2] = 0x55;
	mode_change_buf[3] = 0x0A;

	switch (read_uint8(args, offset)) {
		case 0x01: {
			this->ctrl_mode = MIT;
			ctrl_mode_code = 1;
			break;
		}
		case 0x02: {
			this->ctrl_mode = POSITION_WITH_SPEED_LIMIT;
			ctrl_mode_code = 2;
			break;
		}
		case 0x03: {
			this->ctrl_mode = SPEED_DM;
			ctrl_mode_code = 3;
			break;
		}
		default: {
		}
	}

	memcpy(mode_change_buf + 4, &ctrl_mode_code, 4);

	this->can_id_type = FDCAN_STANDARD_ID;
	this->tx_header.IdType = FDCAN_STANDARD_ID;
	this->tx_header.TxFrameType = FDCAN_DATA_FRAME;
	this->tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	this->tx_header.BitRateSwitch = FDCAN_BRS_OFF;
	this->tx_header.FDFormat = FDCAN_CLASSIC_CAN;
	this->tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	this->tx_header.MessageMarker = 0;
	init_can();
}

void App_DMMotor::collect_outputs(uint8_t *output, int *output_offset) {
	memcpy(output + *(output_offset), report_buf, 8);
	(*output_offset) += 8;
	// isonline?
	output[*output_offset] = (HAL_GetTick() - last_recv_time < 1000 && mode_set == 1) ? 1 : 0;
	(*output_offset) += 1;
}

void App_DMMotor::collect_inputs(uint8_t *input, int *input_offset) {
	this->cmd_motor_enable = read_uint8(input, input_offset);
	switch (this->ctrl_mode) {
		case MIT:
		case POSITION_WITH_SPEED_LIMIT: {
			memcpy(cmd, input + *(input_offset), 8);
			(*input_offset) += 8;
			break;
		}
		case SPEED_DM: {
			memcpy(cmd, input + *(input_offset), 4);
			(*input_offset) += 4;
			break;
		}
		default: {
		}
	}
}

uint32_t App_DMMotor::get_ctrl_packet_id() {
	switch (ctrl_mode) {
		case MIT: {
			return this->ctrl_id;
		}
		case POSITION_WITH_SPEED_LIMIT: {
			return this->ctrl_id + 0x100;
		}
		case SPEED_DM: {
			return this->ctrl_id + 0x200;
		}
		default: {
			return this->ctrl_id;
		}
	}
}

void App_DMMotor::run_task() {
	if (!running) {
		return;
	}

	if (last_recv_time == 0 || HAL_GetTick() - last_recv_time > 1000) {
		memset(this->tx_data, 0xff, 7);
		this->tx_data[7] = 0xfd;
		this->tx_header.Identifier = get_ctrl_packet_id();
		this->tx_header.DataLength = 8;
		HAL_FDCAN_AddMessageToTxFifoQ(this->can_inst, &this->tx_header, this->tx_data);
		return;
	}

	if (mode_set == 0 && last_recv_time != 0) {
		this->tx_header.Identifier = 0x7ff;
		this->tx_header.DataLength = 8;
		HAL_FDCAN_AddMessageToTxFifoQ(this->can_inst, &this->tx_header, this->mode_change_buf);
		return;
	}

	if (cmd_motor_enable == 0) {
		memset(this->tx_data, 0xff, 7);
		this->tx_data[7] = 0xfd;
		this->tx_header.Identifier = get_ctrl_packet_id();
		this->tx_header.DataLength = 8;
		HAL_FDCAN_AddMessageToTxFifoQ(this->can_inst, &this->tx_header, this->tx_data);

		if (this->motor_enabled == 1) {
			if (last_state_change_ts == 0) {
				last_state_change_ts = HAL_GetTick();
			}
		}
		return;
	}

	// below cmd enable = 1

	if (this->motor_enabled == 0) {
		memset(this->tx_data, 0xff, 7);
		this->tx_data[7] = 0xfc;
		this->tx_header.Identifier = get_ctrl_packet_id();
		this->tx_header.DataLength = 8;
		HAL_FDCAN_AddMessageToTxFifoQ(this->can_inst, &this->tx_header, this->tx_data);
		if (last_state_change_ts == 0) {
			last_state_change_ts = HAL_GetTick();
		}
		return;
	}

	// below motor is enabled
	switch (this->ctrl_mode) {
		case MIT:
		case POSITION_WITH_SPEED_LIMIT: {
			this->tx_header.DataLength = 8;
			memcpy(tx_data, cmd, 8);
			break;
		}
		case SPEED_DM: {
			this->tx_header.DataLength = 4;
			memcpy(tx_data, cmd, 4);
			break;
		}
		default: {
		}
	}

	this->tx_header.Identifier = get_ctrl_packet_id();
	HAL_FDCAN_AddMessageToTxFifoQ(this->can_inst, &this->tx_header, this->tx_data);
}


void App_DMMotor::can_recv(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data) {
	if (master_id == rx_header->Identifier) {
		if ((rx_data[0] & 0x0F) != (ctrl_id & 0x0F)) {
			return;
		}

		last_recv_time = HAL_GetTick();
		if (this->cmd_motor_enable == 1) {
			if (this->motor_enabled == 0) {
				if (HAL_GetTick() > last_state_change_ts && last_state_change_ts != 0) {
					last_state_change_ts = 0;
					this->motor_enabled = 1;
				}
			}
		}
		if (this->cmd_motor_enable == 0) {
			if (this->motor_enabled == 1) {
				if (HAL_GetTick() > last_state_change_ts && last_state_change_ts != 0) {
					last_state_change_ts = 0;
					this->motor_enabled = 0;
				}
			}
		}
		if (mode_set == 0) {
			if (memcmp(rx_data, mode_change_buf, 8) == 0) {
				mode_set = 1;
				return;
			}
		}
		memcpy(report_buf, rx_data, 8);
	}
}

void App_DMMotor::exit() {
	deinit_can();
}
