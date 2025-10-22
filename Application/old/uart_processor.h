//
// Created by Hang XU on 04/06/2025.
//

#ifndef UART_PROCESSOR_H
#define UART_PROCESSOR_H

#include "usart.h"

void call_uart_recv_to_idle_dma(UART_HandleTypeDef *huart, uint16_t Size);

void call_uart_send_by_dma(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t Size);

void init_uart_buf();

void reset_usart1();

#endif //UART_PROCESSOR_H