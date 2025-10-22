//
// Created by Hang XU on 21/10/2025.
//

#include "initializer.h"
#include "peripheral_manager.hpp"
#include "buffer_manager.hpp"
#include "settings.h"
#include "soes_application.h"
#include "task_manager.h"

extern "C" {
    #include "ecat_slv.h"
    #include "tim.h"
}

extern esc_cfg_t config;

void initialize() {
    init_buffer_manager();
    init_peripheral_manager();
    init_task_manager();
    init_soes_buffers();

    // 10khz
    __HAL_TIM_SET_PRESCALER(&htim17, LED_PSC);
    // task not loaded, 4hz / 250ms
    __HAL_TIM_SET_AUTORELOAD(&htim17, LED_250MS_ARR);
    HAL_TIM_Base_Start_IT(&htim17);

    ecat_slv_init(&config);
    // ecat esc initialized
    HAL_GPIO_WritePin(LED_LBOARD_1_GPIO_Port, LED_LBOARD_1_Pin, GPIO_PIN_RESET);
}
