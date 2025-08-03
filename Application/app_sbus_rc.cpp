//
// Created by Hang XU on 02/08/2025.
//
#include "task_defs.h"
#include "uart_processor.h"
#include <cstring>

extern "C" {
#include "usart.h"
#include "device_conf.h"
}

App_SBUS_RC::App_SBUS_RC(uint8_t *args, int *offset) {
    this->uart_inst = &huart8;
    init_uart8();
    call_uart_recv_to_idle_dma(this->uart_inst, 25);
}

void App_SBUS_RC::collect_outputs(uint8_t *output, int *output_offset) {
    rc_buf[25] = HAL_GetTick() - rc_last_receive_time <= 20;
    memcpy(output + *(output_offset), rc_buf + 1, 23);
    (*output_offset) += 23;
    memcpy(output + *(output_offset), rc_buf + 25, 1);
    (*output_offset) += 1;
}

void App_SBUS_RC::uart_recv(uint16_t size, uint8_t *rx_data) {
    if (size == 25) {
        if (rx_data[0] == 0x0F && rx_data[24] == 0x00) {
            rc_last_receive_time = HAL_GetTick();
            memcpy(rc_buf, rx_data, 25);
        }
    }
    call_uart_recv_to_idle_dma(this->uart_inst, 25);
}

void App_SBUS_RC::uart_recv_err() {
    call_uart_recv_to_idle_dma(this->uart_inst, 25);
}

void App_SBUS_RC::exit() {
    deinit_uart8();
}
