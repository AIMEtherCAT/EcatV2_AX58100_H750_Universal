//
// Created by Hang XU on 04/06/2025.
//

#include "device_conf.h"
#include "stm32h7xx.h"
#include "fdcan.h"
#include "i2c.h"
#include "usart.h"
#include "tim.h"

uint8_t can2_inited = 0;
uint8_t usart1_inited = 0;
uint8_t uart4_inited = 0;
uint8_t uart8_inited = 0;
uint8_t tim3_inited = 0;
uint8_t tim2_inited = 0;
uint8_t i2c3_inited = 0;

void init_can2() {
    if (can2_inited) {
        return;
    }

    MX_FDCAN2_Init();

    FDCAN_FilterTypeDef can_filter_st;
    can_filter_st.IdType = FDCAN_STANDARD_ID;
    can_filter_st.FilterIndex = 0;
    can_filter_st.FilterType = FDCAN_FILTER_MASK;
    can_filter_st.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    can_filter_st.FilterID1 = 0x00000000;
    can_filter_st.FilterID2 = 0x00000000;

    HAL_FDCAN_ConfigFilter(&hfdcan2, &can_filter_st);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_Start(&hfdcan2);
    can2_inited = 1;
}

void deinit_can2() {
    if (!can2_inited) {
        return;
    }
    HAL_FDCAN_DeInit(&hfdcan2);
    can2_inited = 0;
}

void init_usart1() {
    if (usart1_inited) {
        return;
    }

    MX_USART1_UART_Init();
    usart1_inited = 1;
}

void deinit_usart1() {
    if (!usart1_inited) {
        return;
    }
    HAL_UART_DeInit(&huart1);
    usart1_inited = 0;
}

void init_uart4() {
    if (uart4_inited) {
        return;
    }

    MX_UART4_Init();
    uart4_inited = 1;
}

void deinit_uart4() {
    if (!uart4_inited) {
        return;
    }
    HAL_UART_DeInit(&huart4);
    uart4_inited = 0;
}

void init_uart8() {
    if (uart8_inited) {
        return;
    }

    MX_UART8_Init();
    uart8_inited = 1;
}

void deinit_uart8() {
    if (!uart8_inited) {
        return;
    }
    HAL_UART_DeInit(&huart8);
    uart8_inited = 0;
}

void init_tim3() {
    if (tim3_inited) {
        return;
    }
    MX_TIM3_Init();
    tim3_inited = 1;
}

void deinit_tim3() {
    if (!tim3_inited) {
        return;
    }
    HAL_TIM_PWM_DeInit(&htim3);
}

void init_tim2() {
    if (tim2_inited) {
        return;
    }
    MX_TIM2_Init();
    tim2_inited = 1;
}

void deinit_tim2() {
    if (!tim2_inited) {
        return;
    }
    HAL_TIM_PWM_DeInit(&htim2);
}

void init_i2c3() {
    if (i2c3_inited) {
        return;
    }
    MX_I2C3_Init();
}

void deinit_i2c3() {
    if (!i2c3_inited) {
        return;
    }
    HAL_I2C_DeInit(&hi2c3);
}