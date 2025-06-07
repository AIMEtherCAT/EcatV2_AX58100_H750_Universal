//
// Created by Hang XU on 04/06/2025.
//
#include "task_defs.h"
#include <cstring>
#include "IOUtils.h"
#include "dshot.h"

extern "C" {
#include "tim.h"
#include "device_conf.h"
}

App_DSHOT::App_DSHOT(uint8_t *args, int *offset) {
    dshot_id = read_uint8(args, offset);
    if (dshot_id == 1) {
        init_tim3();
        init_dshot1();
        write_dshot1(0, 0, 0, 0);
    } else if (dshot_id == 2) {
        init_tim2();
        init_dshot2();
        write_dshot2(0, 0, 0, 0);
    }
}

void App_DSHOT::collect_inputs(uint8_t *input, int *input_offset) {
    if (dshot_id == 1) {
        write_dshot1(
            read_uint16(input, input_offset),
            read_uint16(input, input_offset),
            read_uint16(input, input_offset),
            read_uint16(input, input_offset)
        );
    } else if (dshot_id == 2) {
        write_dshot2(
            read_uint16(input, input_offset),
            read_uint16(input, input_offset),
            read_uint16(input, input_offset),
            read_uint16(input, input_offset)
        );
    }
}

void App_DSHOT::exit() {
    if (dshot_id == 1) {
        deinit_tim3();

        __HAL_TIM_DISABLE_DMA(DSHOT1_TIM, TIM_DMA_CC1);
        __HAL_TIM_DISABLE_DMA(DSHOT1_TIM, TIM_DMA_CC1);
        __HAL_TIM_DISABLE_DMA(DSHOT1_TIM, TIM_DMA_CC1);
        __HAL_TIM_DISABLE_DMA(DSHOT1_TIM, TIM_DMA_CC1);

        DSHOT1_TIM->hdma[TIM_DMA_ID_CC1]->XferCpltCallback = NULL;
        DSHOT1_TIM->hdma[TIM_DMA_ID_CC2]->XferCpltCallback = NULL;
        DSHOT1_TIM->hdma[TIM_DMA_ID_CC3]->XferCpltCallback = NULL;
        DSHOT1_TIM->hdma[TIM_DMA_ID_CC4]->XferCpltCallback = NULL;

        HAL_TIM_PWM_Stop(DSHOT1_TIM, DSHOT1_MOTOR_1_TIM_CHANNEL);
        HAL_TIM_PWM_Stop(DSHOT1_TIM, DSHOT1_MOTOR_2_TIM_CHANNEL);
        HAL_TIM_PWM_Stop(DSHOT1_TIM, DSHOT1_MOTOR_3_TIM_CHANNEL);
        HAL_TIM_PWM_Stop(DSHOT1_TIM, DSHOT1_MOTOR_4_TIM_CHANNEL);
    } else if (dshot_id == 2) {
        deinit_tim2();

        __HAL_TIM_DISABLE_DMA(DSHOT2_TIM, TIM_DMA_CC1);
        __HAL_TIM_DISABLE_DMA(DSHOT2_TIM, TIM_DMA_CC1);
        __HAL_TIM_DISABLE_DMA(DSHOT2_TIM, TIM_DMA_CC1);
        __HAL_TIM_DISABLE_DMA(DSHOT2_TIM, TIM_DMA_CC1);

        DSHOT2_TIM->hdma[TIM_DMA_ID_CC1]->XferCpltCallback = NULL;
        DSHOT2_TIM->hdma[TIM_DMA_ID_CC2]->XferCpltCallback = NULL;
        DSHOT2_TIM->hdma[TIM_DMA_ID_CC3]->XferCpltCallback = NULL;
        DSHOT2_TIM->hdma[TIM_DMA_ID_CC4]->XferCpltCallback = NULL;

        HAL_TIM_PWM_Stop(DSHOT2_TIM, DSHOT2_MOTOR_1_TIM_CHANNEL);
        HAL_TIM_PWM_Stop(DSHOT2_TIM, DSHOT2_MOTOR_2_TIM_CHANNEL);
        HAL_TIM_PWM_Stop(DSHOT2_TIM, DSHOT2_MOTOR_3_TIM_CHANNEL);
        HAL_TIM_PWM_Stop(DSHOT2_TIM, DSHOT2_MOTOR_4_TIM_CHANNEL);
    }
}

