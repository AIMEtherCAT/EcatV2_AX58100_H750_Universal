//
// Created by Hang XU on 04/07/2025.
//
#include "../task_defs.h"
#include <cstring>
#include "io_utils.h"
#include "dshot.h"

extern "C" {
#include "fdcan.h"
#include "peripheral_conf.hpp"
}

App_CAN_PMU::App_CAN_PMU(uint8_t *args, int *offset) {
    this->can_inst = &hfdcan2;
    this->can_id_type = FDCAN_EXTENDED_ID;
    this->can_tx_header.IdType = FDCAN_EXTENDED_ID;
    this->can_tx_header.TxFrameType = FDCAN_DATA_FRAME;
    this->can_tx_header.DataLength = 8;
    this->can_tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    this->can_tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    this->can_tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    this->can_tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    this->can_tx_header.MessageMarker = 0;
    this->can_tx_header.Identifier = 0x1401557F;
    init_can();
    uptime = 1;
    transfer_id = 0;
    memset(pmu_buffer, 0, 8);
}

void App_CAN_PMU::collect_inputs(uint8_t *output, int *output_offset) {
    if (HAL_GetTick() - last_node_status_pub_ts > 998) {
        uptime++;

        node_status_data[0] = (uint8_t)(uptime & 0xFF);
        node_status_data[1] = (uint8_t)((uptime >> 8) & 0xFF);
        node_status_data[2] = (uint8_t)((uptime >> 16) & 0xFF);
        node_status_data[3] = (uint8_t)((uptime >> 24) & 0xFF);
        node_status_data[4] = 0x00;
        node_status_data[5] = 0x00;
        node_status_data[6] = 0x00;
        node_status_data[7] = 0x00;
        node_status_data[7] = 0xE0 | (transfer_id & 0x1F);
        transfer_id = (transfer_id + 1) % 32;
        HAL_FDCAN_AddMessageToTxFifoQ(this->can_inst, &can_tx_header, node_status_data);
        last_node_status_pub_ts = HAL_GetTick();
    }
}

void App_CAN_PMU::collect_outputs(uint8_t *output, int *output_offset) {
    int off = 0;
    for (int i = 0; i < 3; i++) {
        if (read_float16(pmu_buffer, &off) > 1e-2) {
            memcpy(output + *(output_offset), pmu_buffer + i * 2, 2);
        }
        (*output_offset) += (2);
    }
}

void App_CAN_PMU::can_recv(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data) {
    if (rx_header->DataLength < 1 || rx_header->DataLength > 8) return;

    UAVCAN_TailByte_t tail = parse_tail_byte(rx_data[rx_header->DataLength - 1]);
    uint32_t now = HAL_GetTick();
    uint8_t is_first = tail.start;
    uint8_t is_last = tail.end;

    if (!uavcan_rx.initialized ||
        (now - uavcan_rx.last_ts > UAVCAN_TID_TIMEOUT_MS) ||
        (is_first && ((uint8_t)((tail.tid - uavcan_rx.transfer_id) & 0x1F) > 1))) {
        uavcan_rx.initialized = 1;
        uavcan_rx.transfer_id = tail.tid;
        uavcan_rx.toggle = 0;
        uavcan_rx.len = 0;
        uavcan_rx.crc = 0xFFFF;

        if (!is_first) {
            uavcan_rx.transfer_id = (tail.tid + 1) & 0x1F;
            return;
        }
    }

    if (tail.tid != uavcan_rx.transfer_id)
        return;

    if (tail.toggle != uavcan_rx.toggle)
        return;

    uavcan_rx.last_ts = now;
    uavcan_rx.toggle ^= 1;

    uint8_t data_len = rx_header->DataLength - 1;
    const uint8_t *p = rx_data;

    if (is_first) {
        if (data_len < 2) return;
        uavcan_rx.crc = p[0] | (p[1] << 8);
        p += 2;
        data_len -= 2;
    }

    if (uavcan_rx.len + data_len > UAVCAN_BUF_SIZE) {
        uavcan_rx.len = 0;
        return;
    }

    for (uint8_t i = 0; i < data_len; ++i) {
        uavcan_rx.buffer[uavcan_rx.len++] = p[i];
    }

    if (is_last) {
        if (uavcan_rx.buffer[0] != 0) {
            memcpy(pmu_buffer, uavcan_rx.buffer, 6);
        }

        uavcan_rx.len = 0;
        uavcan_rx.toggle = 0;
        uavcan_rx.transfer_id = (uavcan_rx.transfer_id + 1) & 0x1F;
        memset(uavcan_rx.buffer, 0, 64);
    }
}

void App_CAN_PMU::exit() {
    deinit_can();
}