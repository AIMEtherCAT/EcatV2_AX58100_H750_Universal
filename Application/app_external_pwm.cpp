//
// Created by Hang XU on 06/06/2025.
//
#include <cfloat>

#include "task_defs.h"
#include <cstring>

#include "CRC8_CRC16.h"
#include "IOUtils.h"
#include "dshot.h"
#include "uart_processor.h"
extern "C" {
#include "usart.h"
#include "device_conf.h"
}

App_External_PWM::App_External_PWM(uint8_t *args, int *offset) {
    uart_id = read_uint8(args, offset);
    if (uart_id == 4) {
        uart_inst = &huart4;
        init_uart4();
    } else if (uart_id == 1) {
        uart_inst = &huart1;
        init_usart1();
    }
    expected_period = read_uint16(args, offset);
    enabled_channel_count = read_uint8(args, offset);
    uint16_t init_value = read_uint16(args, offset);

    cmd.header = 0x01;
    cmd.expected_period = expected_period;
    for (int i=0;i<16;i++) {
        if (i < enabled_channel_count) {
            cmd.servo_cmd[i] = init_value;
        } else {
            cmd.servo_cmd[i] = 0;
        }
    }
    memcpy(buf, &cmd, 37);
    append_CRC16_check_sum(buf, 37);
    call_uart_send_by_dma(uart_inst, buf, 37);
}

void App_External_PWM::collect_inputs(uint8_t *input, int *input_offset) {
    cmd.header = 0x01;
    cmd.expected_period = expected_period;
    for (int i=0;i<16;i++) {
        if (i < enabled_channel_count) {
            cmd.servo_cmd[i] = read_uint16(input, input_offset);
        } else {
            cmd.servo_cmd[i] = 0;
        }
    }
    memcpy(buf, &cmd, 37);
    append_CRC16_check_sum(buf, 37);
    call_uart_send_by_dma(uart_inst, buf, 37);
}

void App_External_PWM::exit() {
    HAL_UART_Abort(uart_inst);
    if (uart_id == 1) {
        deinit_usart1();
    } else if (uart_id == 4) {
        deinit_uart4();
    }
}
