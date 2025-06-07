//
// Created by Hang XU on 04/06/2025.
//
#include "task_defs.h"
#include "uart_processor.h"
#include <cstring>

extern "C" {
#include "usart.h"
#include "device_conf.h"
}

App_DJI_RC::App_DJI_RC(uint8_t *args, int *offset) {
    this->uart_inst = &huart8;
    init_uart8();
    call_uart_recv_to_idle_dma(this->uart_inst, 18);
}

void App_DJI_RC::collect_outputs(uint8_t *output, int *output_offset) {
    rc_buf[18] = HAL_GetTick() - rc_last_receive_time <= 20;
    memcpy(output + *(output_offset), rc_buf, 19);
    (*output_offset) += 19;
}

void App_DJI_RC::uart_recv(uint16_t size, uint8_t *rx_data) {
    if (size == 18) {
        uint16_t channel = (rx_data[0] | (rx_data[1] << 8)) & 0x07ff;
        if (fabs(channel) <= RC_CHANNAL_ERROR_VALUE) {
            rc_last_receive_time = HAL_GetTick();
            memcpy(rc_buf, rx_data, 18);
        } else {
            memset(rc_buf, 0, sizeof(rc_buf));
        }
    }
    call_uart_recv_to_idle_dma(this->uart_inst, 18);
}

void App_DJI_RC::uart_recv_err() {
    call_uart_recv_to_idle_dma(this->uart_inst, 18);
}

void App_DJI_RC::exit() {
    deinit_uart8();
}
