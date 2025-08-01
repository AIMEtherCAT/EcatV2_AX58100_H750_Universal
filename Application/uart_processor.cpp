//
// Created by Hang XU on 04/06/2025.
//

#include "uart_processor.h"

#include <cstring>
#include <vector>
#include "task_defs.h"

extern "C" {
#include "usart.h"
}

DMA_BUFFER uint8_t uart8_buf[512];
DMA_BUFFER uint8_t uart4_buf[512];
DMA_BUFFER uint8_t usart1_buf[512];
DMA_BUFFER uint8_t uart8_send_buf[512];
DMA_BUFFER uint8_t uart4_send_buf[512];
DMA_BUFFER uint8_t usart1_send_buf[512];
extern std::vector<UartRunnable *> uart_list;

void init_uart_buf() {
    memset(uart8_buf, 0, 512);
    memset(uart4_buf, 0, 512);
    memset(usart1_buf, 0, 512);
    memset(uart8_send_buf, 0, 512);
    memset(uart4_send_buf, 0, 512);
    memset(usart1_send_buf, 0, 512);
}

uint8_t usart1_ready = 1;
uint8_t uart4_ready = 1;
uint8_t uart8_ready = 1;

void reset_usart1() {
    HAL_DMA_Abort(huart1.hdmatx);
    usart1_ready = 1;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        usart1_ready = 1;
    } else if (huart->Instance == UART4) {
        uart4_ready = 1;
    } else if (huart->Instance == UART8) {
        uart8_ready = 1;
    }

    for (UartRunnable *runnable: uart_list) {
        if (huart->Instance != runnable->uart_inst->Instance) {
            continue;
        }

        runnable->uart_dma_tx_finished_callback();
    }
}

void call_uart_send_by_dma(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t Size) {
    if (huart->Instance == USART1) {
        if (usart1_ready == 0) {
            return;
        }
        memcpy(usart1_send_buf, buf, Size);
        HAL_UART_Transmit_DMA(&huart1, usart1_send_buf, Size);
        usart1_ready = 0;
    } else if (huart->Instance == UART4) {
        if (uart4_ready == 0) {
            return;
        }
        memcpy(uart4_send_buf, buf, Size);
        HAL_UART_Transmit_DMA(&huart4, uart4_send_buf, Size);
        uart4_ready = 0;
    } else if (huart->Instance == UART8) {
        if (uart8_ready == 0) {
            return;
        }
        memcpy(uart8_send_buf, buf, Size);
        HAL_UART_Transmit_DMA(&huart8, uart8_send_buf, Size);
        uart8_ready = 0;
    }
}

void call_uart_recv_to_idle_dma(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART1) {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, usart1_buf, Size);
    } else if (huart->Instance == UART4) {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, uart4_buf, Size);
    } else if (huart->Instance == UART8) {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart8, uart8_buf, Size);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    for (UartRunnable *runnable: uart_list) {
        if (huart->Instance != runnable->uart_inst->Instance) {
            continue;
        }

        if (huart->Instance == USART1) {
            runnable->uart_recv(Size, usart1_buf);
        } else if (huart->Instance == UART4) {
            runnable->uart_recv(Size, uart4_buf);
        } else if (huart->Instance == UART8) {
            runnable->uart_recv(Size, uart8_buf);
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    for (UartRunnable *runnable: uart_list) {
        if (huart->Instance != runnable->uart_inst->Instance) {
            continue;
        }

        runnable->uart_recv_err();
    }
}
