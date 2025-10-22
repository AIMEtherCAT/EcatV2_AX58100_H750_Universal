//
// Created by Hang XU on 21/10/2025.
//

#include "initializer.h"
#include "peripheral_manager.hpp"
#include "buffer_manager.hpp"
#include "settings.h"
#include "soes_application.hpp"
#include "task_manager.hpp"

extern "C" {
#include "ecat_slv.h"
#include "tim.h"
}

extern esc_cfg_t config;

void initialize() {
    aim::io::buffer::init_buffer_manager();
    aim::hardware::peripheral::init_peripheral_manager();
    aim::ecat::task::init_task_manager();
    aim::ecat::application::init_soes_env();

    // 10khz
    __HAL_TIM_SET_PRESCALER(&htim17, aim::ecat::task::LED_PSC);
    __HAL_TIM_SET_AUTORELOAD(&htim17, aim::ecat::task::LED_TASK_NOT_LOADED_ARR);
    HAL_TIM_Base_Start_IT(&htim17);

    ecat_slv_init(&config);
    // ecat esc initialized
    HAL_GPIO_WritePin(LED_LBOARD_1_GPIO_Port, LED_LBOARD_1_Pin, GPIO_PIN_RESET);
}