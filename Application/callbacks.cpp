//
// Created by Hang XU on 21/10/2025.
//
#include "peripheral_manager.hpp"
#include "task_manager.hpp"

using namespace aim::ecat::task;
using namespace aim::hardware;

// ReSharper disable once CppParameterMayBeConstPtrOrRef
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        static_cast<peripheral::UartPeripheral *>(get_peripheral(peripheral::Type::PERIPHERAL_USART1))->is_busy.clear(); // NOLINT
    } else if (huart->Instance == UART4) {
        static_cast<peripheral::UartPeripheral *>(get_peripheral(peripheral::Type::PERIPHERAL_UART4))->is_busy.clear(); // NOLINT
    } else if (huart->Instance == UART8) {
        static_cast<peripheral::UartPeripheral *>(get_peripheral(peripheral::Type::PERIPHERAL_UART8))->is_busy.clear(); // NOLINT
    }

    for (const std::shared_ptr<runnable_conf> &conf: *get_run_confs()) {
        if (!conf->is_uart_task.get()) {
            continue;
        }
        if (huart->Instance != conf->runnable->get_peripheral<peripheral::UartPeripheral>()->_huart->Instance) {
            continue;
        }

        static_cast<UartRunnable *>(conf->runnable.get())->uart_dma_tx_finished_callback(); // NOLINT
    }
}

// ReSharper disable once CppParameterMayBeConstPtrOrRef
// ReSharper disable once CppParameterMayBeConst
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    for (const std::shared_ptr<runnable_conf> &conf: *get_run_confs()) {
        if (!conf->is_uart_task.get()) {
            continue;
        }
        if (huart->Instance != conf->runnable->get_peripheral<peripheral::UartPeripheral>()->_huart->Instance) {
            continue;
        }

        conf->runnable->get_peripheral<peripheral::UartPeripheral>()->_recv_buf->reset_index();
        static_cast<UartRunnable *>(conf->runnable.get())->uart_recv(Size); // NOLINT
    }
}

// ReSharper disable once CppParameterMayBeConstPtrOrRef
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        static_cast<peripheral::UartPeripheral *>(get_peripheral(peripheral::Type::PERIPHERAL_USART1))->is_busy.clear(); // NOLINT
    } else if (huart->Instance == UART4) {
        static_cast<peripheral::UartPeripheral *>(get_peripheral(peripheral::Type::PERIPHERAL_UART4))->is_busy.clear(); // NOLINT
    } else if (huart->Instance == UART8) {
        static_cast<peripheral::UartPeripheral *>(get_peripheral(peripheral::Type::PERIPHERAL_UART8))->is_busy.clear(); // NOLINT
    }

    for (const std::shared_ptr<runnable_conf> &conf: *get_run_confs()) {
        if (!conf->is_uart_task.get()) {
            continue;
        }
        if (huart->Instance != conf->runnable->get_peripheral<peripheral::UartPeripheral>()->_huart->Instance) {
            continue;
        }

        conf->runnable->get_peripheral<peripheral::UartPeripheral>()->_recv_buf->reset();
        conf->runnable->get_peripheral<peripheral::UartPeripheral>()->_send_buf->reset();
        static_cast<UartRunnable *>(conf->runnable.get())->uart_err(); // NOLINT
    }
}

void process_can_data(const FDCAN_HandleTypeDef *hfdcan, FDCAN_RxHeaderTypeDef *rx_header, uint8_t rx_data[8]) {
    for (const std::shared_ptr<runnable_conf> &conf: *get_run_confs()) {
        if (!conf->is_can_task.get()) {
            continue;
        }
        if (hfdcan->Instance != static_cast<CanRunnable *>(conf->runnable.get())->can_inst_->Instance) { // NOLINT
            continue;
        }
        if (rx_header->IdType != static_cast<CanRunnable *>(conf->runnable.get())->can_id_type_) { // NOLINT
            continue;
        }

        static_cast<CanRunnable *>(conf->runnable.get())->can_recv(rx_header, rx_data); // NOLINT
    }
}

// ReSharper disable once CppParameterMayBeConst
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t /* RxFifo0ITs */) {
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
    process_can_data(hfdcan, &rx_header, rx_data);
}

// ReSharper disable once CppParameterMayBeConst
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t /* RxFifo1ITs */) {
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rx_header, rx_data);
    process_can_data(hfdcan, &rx_header, rx_data);
}
