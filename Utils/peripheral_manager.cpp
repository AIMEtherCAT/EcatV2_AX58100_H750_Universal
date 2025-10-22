//
// Created by Hang XU on 04/06/2025.
//

#include "peripheral_manager.hpp"

#include <map>
#include <memory>

extern "C" {
#include "fdcan.h"
#include "i2c.h"
#include "usart.h"
#include "tim.h"
#include "adc.h"
#include "cmsis_os.h"
#include "stm32h7xx.h"

extern osMutexId ADC1InitMutexHandle;
extern osMutexId CANInitMutexHandle;
extern osMutexId USART1InitMutexHandle;
extern osMutexId UART4InitMutexHandle;
extern osMutexId UART8InitMutexHandle;
extern osMutexId TIM2InitMutexHandle;
extern osMutexId TIM3InitMutexHandle;
extern osMutexId I2C3InitMutexHandle;
}

namespace aim::hardware::peripheral {
    void CANPeripheral::_init_impl() {
        FDCAN_FilterTypeDef can_filter;

        can_filter.IdType = FDCAN_STANDARD_ID;
        can_filter.FilterIndex = 0;
        can_filter.FilterType = FDCAN_FILTER_MASK;
        can_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        can_filter.FilterID1 = 0x00000000;
        can_filter.FilterID2 = 0x00000000;

        MX_FDCAN1_Init();
        HAL_FDCAN_ConfigFilter(&hfdcan1, &can_filter);
        HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
        HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
        HAL_FDCAN_Start(&hfdcan1);


        MX_FDCAN2_Init();
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

        HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_ACCEPT_IN_RX_FIFO1, FDCAN_ACCEPT_IN_RX_FIFO1,
                                     FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
        HAL_FDCAN_ConfigInterruptLines(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, FDCAN_INTERRUPT_LINE1);
        HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
        HAL_FDCAN_Start(&hfdcan2);
    }

    void CANPeripheral::_deinit_impl() {
        HAL_FDCAN_DeactivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
        HAL_FDCAN_DeInit(&hfdcan1);

        HAL_FDCAN_DeactivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE);
        HAL_FDCAN_DeInit(&hfdcan2);
    }

    void USART1Peripheral::_init_impl() {
        MX_USART1_UART_Init();
    }

    void USART1Peripheral::_deinit_impl() {
        HAL_UART_DeInit(&huart1);
    }

    void UART4Peripheral::_init_impl() {
        MX_UART4_Init();
    }

    void UART4Peripheral::_deinit_impl() {
        HAL_UART_DeInit(&huart4);
    }

    void UART8Peripheral::_init_impl() {
        MX_UART8_Init();
    }

    void UART8Peripheral::_deinit_impl() {
        HAL_UART_DeInit(&huart8);
    }

    void TIM2Peripheral::_init_impl() {
        MX_TIM2_Init();
    }

    void TIM2Peripheral::_deinit_impl() {
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
        HAL_TIM_PWM_DeInit(&htim2);
    }

    void TIM3Peripheral::_init_impl() {
        MX_TIM3_Init();
    }

    void TIM3Peripheral::_deinit_impl() {
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
        HAL_TIM_PWM_DeInit(&htim3);
    }

    void I2C3Peripheral::_init_impl() {
        MX_I2C3_Init();
    }

    void I2C3Peripheral::_deinit_impl() {
        HAL_I2C_DeInit(&hi2c3);
    }

    void ADC1Peripheral::_init_impl() {
        MX_ADC1_Init();
        HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
        HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);
    }

    void ADC1Peripheral::_deinit_impl() {
        HAL_ADC_DeInit(&hadc1);
    }

    static std::map<Type, std::unique_ptr<Peripheral> > instances;

    void init_peripheral_manager() {
        if (!instances.empty()) return;

        osMutexDef(ADC1InitMutex);
        ADC1InitMutexHandle = osMutexCreate(osMutex(ADC1InitMutex));
        osMutexDef(CANInitMutex);
        CANInitMutexHandle = osMutexCreate(osMutex(CANInitMutex));
        osMutexDef(USART1InitMutex);
        USART1InitMutexHandle = osMutexCreate(osMutex(USART1InitMutex));
        osMutexDef(UART4InitMutex);
        UART4InitMutexHandle = osMutexCreate(osMutex(UART4InitMutex));
        osMutexDef(UART8InitMutex);
        UART8InitMutexHandle = osMutexCreate(osMutex(UART8InitMutex));
        osMutexDef(TIM2InitMutex);
        TIM2InitMutexHandle = osMutexCreate(osMutex(TIM2InitMutex));
        osMutexDef(TIM3InitMutex);
        TIM3InitMutexHandle = osMutexCreate(osMutex(TIM3InitMutex));
        osMutexDef(I2C3InitMutex);
        I2C3InitMutexHandle = osMutexCreate(osMutex(I2C3InitMutex));

        instances[Type::PERIPHERAL_USART1] = std::make_unique<USART1Peripheral>(USART1InitMutexHandle,
            &huart1, buffer::get_buffer(buffer::Type::USART1_SEND), buffer::get_buffer(buffer::Type::USART1_RECV));
        instances[Type::PERIPHERAL_UART4] = std::make_unique<UART4Peripheral>(UART4InitMutexHandle,
                                                                              &huart4,
                                                                              buffer::get_buffer(
                                                                                  buffer::Type::UART4_SEND),
                                                                              buffer::get_buffer(
                                                                                  buffer::Type::UART4_RECV));
        instances[Type::PERIPHERAL_UART8] = std::make_unique<UART8Peripheral>(UART8InitMutexHandle,
                                                                              &huart8,
                                                                              buffer::get_buffer(
                                                                                  buffer::Type::UART8_SEND),
                                                                              buffer::get_buffer(
                                                                                  buffer::Type::UART8_RECV));
        instances[Type::PERIPHERAL_I2C3] = std::make_unique<I2C3Peripheral>(I2C3InitMutexHandle);
        instances[Type::PERIPHERAL_TIM2] = std::make_unique<TIM2Peripheral>(TIM2InitMutexHandle);
        instances[Type::PERIPHERAL_TIM3] = std::make_unique<TIM3Peripheral>(TIM3InitMutexHandle);
        instances[Type::PERIPHERAL_CAN] = std::make_unique<CANPeripheral>(CANInitMutexHandle);
    }

    Peripheral *get_peripheral(const Type type) {
        if (const auto it = instances.find(type); it != instances.end()) {
            return it->second.get();
        }
        return nullptr;
    }
}