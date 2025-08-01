//
// Created by Hang XU on 04/06/2025.
//

#include "can_processor.h"

#include <vector>

#include "IOUtils.h"
#include "task_manager.h"
#include "task_defs.h"

extern "C" {
#include "fdcan.h"
}

extern std::vector<CanRunnable *> can_list;

void process_can_data(FDCAN_HandleTypeDef *hcan, FDCAN_RxHeaderTypeDef *rx_header, uint8_t rx_data[8]) {
    for (CanRunnable *runnable: can_list) {
        if (runnable->can_inst == hcan && runnable->can_id_type == rx_header->IdType) {
            runnable->can_recv(rx_header, rx_data);
        }
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hcan, uint32_t RxFifo0ITs) {
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_FDCAN_GetRxMessage(hcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
    process_can_data(hcan, &rx_header, rx_data);
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hcan, uint32_t RxFifo1ITs) {
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_FDCAN_GetRxMessage(hcan, FDCAN_RX_FIFO1, &rx_header, rx_data);
    process_can_data(hcan, &rx_header, rx_data);
}
