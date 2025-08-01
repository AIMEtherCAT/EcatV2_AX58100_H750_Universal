//
// Created by Hang XU on 04/06/2025.
//

#include "device_conf.h"
#include "stm32h7xx.h"
#include "fdcan.h"
#include "i2c.h"
#include "usart.h"
#include "tim.h"
#include "adc.h"

uint8_t can_inited = 0;
uint8_t usart1_inited = 0;
uint8_t uart4_inited = 0;
uint8_t uart8_inited = 0;
uint8_t tim3_inited = 0;
uint8_t tim2_inited = 0;
uint8_t i2c3_inited = 0;
uint8_t adc_inited = 0;

void init_can() {
    if (can_inited) {
        return;
    }

    MX_FDCAN1_Init();
    MX_FDCAN2_Init();
    FDCAN_FilterTypeDef can_filter;

    can_filter.IdType = FDCAN_STANDARD_ID;
    can_filter.FilterIndex = 0;
    can_filter.FilterType = FDCAN_FILTER_MASK;
    can_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    can_filter.FilterID1 = 0x00000000;
    can_filter.FilterID2 = 0x00000000;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &can_filter);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_Start(&hfdcan1);

    can_filter.IdType = FDCAN_STANDARD_ID;
    can_filter.FilterIndex = 0;
    can_filter.FilterType = FDCAN_FILTER_MASK;
    can_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    can_filter.FilterID1 = 0x00000000;
    can_filter.FilterID2 = 0x00000000;
    HAL_FDCAN_ConfigFilter(&hfdcan2, &can_filter);

    can_filter.IdType = FDCAN_EXTENDED_ID;
    can_filter.FilterIndex = 1;
    can_filter.FilterType = FDCAN_FILTER_MASK;
    can_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    can_filter.FilterID1 = 0x00000000;
    can_filter.FilterID2 = 0x00000000;
    HAL_FDCAN_ConfigFilter(&hfdcan2, &can_filter);

    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_ACCEPT_IN_RX_FIFO1, FDCAN_ACCEPT_IN_RX_FIFO1, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_ConfigInterruptLines(&hfdcan2,
        FDCAN_IT_RX_FIFO1_NEW_MESSAGE,
        FDCAN_INTERRUPT_LINE1);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
    HAL_FDCAN_Start(&hfdcan2);

    can_inited = 1;
}

void deinit_can() {
    if (!can_inited) {
        return;
    }

    HAL_FDCAN_DeInit(&hfdcan1);
    HAL_FDCAN_DeInit(&hfdcan2);

    can_inited = 0;
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
    tim3_inited = 0;
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
    tim2_inited = 0;
}

void init_i2c3() {
    if (i2c3_inited) {
        return;
    }
    MX_I2C3_Init();
    i2c3_inited = 1;
}

void deinit_i2c3() {
    if (!i2c3_inited) {
        return;
    }
    HAL_I2C_DeInit(&hi2c3);
    i2c3_inited = 0;
}

void init_adc() {
    if (adc_inited) {
        return;
    }
    MX_ADC1_Init();
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);
    adc_inited = 1;
}

void deinit_adc() {
    if (!adc_inited) {
        return;
    }
    HAL_ADC_DeInit(&hadc1);
    adc_inited = 0;
}