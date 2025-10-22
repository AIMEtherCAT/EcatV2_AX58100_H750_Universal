//
// Created by Hang XU on 04/06/2025.
//
#include <cfloat>

#include "../task_defs.h"
#include <cstring>
#include "io_utils.h"
#include "dshot.h"

extern "C" {
#include "tim.h"
#include "peripheral_conf.hpp"
}

App_Vanilla_PWM::App_Vanilla_PWM(uint8_t *args, int *offset) {
    tim_id = read_uint8(args, offset);
    if (tim_id == 2) {
        tim_inst = &htim2;
        init_tim2();
    } else if (tim_id == 3) {
        tim_inst = &htim3;
        init_tim3();
    }
    expected_period = read_uint16(args, offset);

    double min_error = DBL_MAX;
    for (uint32_t psc = 0; psc <= 0xFFFF; ++psc) {
        double temp_arr = ((double) expected_period * TIM_FREQ / 1000000.0) / (psc + 1) - 1.0;
        if (temp_arr < 0 || temp_arr > 65535)
            continue;
        double arr = (double) (temp_arr + 0.5);
        double actual_period = ((double) (psc + 1) * (arr + 1)) * 1000000.0 / TIM_FREQ;
        double error = fabs(actual_period - expected_period);
        if (error < min_error) {
            min_error = error;
            best_psc = (uint16_t) psc;
            best_arr = (uint16_t) arr;
        }
        if (error < 1e-6)
            break;
    }

    uint16_t init_value = read_uint16(args, offset);

    __HAL_TIM_SET_PRESCALER(tim_inst, best_psc);
    __HAL_TIM_SET_AUTORELOAD(tim_inst, best_arr);
    cmd[0] = calc_compare(init_value);
    cmd[1] = calc_compare(init_value);
    cmd[2] = calc_compare(init_value);
    cmd[3] = calc_compare(init_value);
}

void App_Vanilla_PWM::collect_inputs(uint8_t *input, int *input_offset) {
    __HAL_TIM_SET_COMPARE(tim_inst, TIM_CHANNEL_1, cmd[0]);
    __HAL_TIM_SET_COMPARE(tim_inst, TIM_CHANNEL_2, cmd[1]);
    __HAL_TIM_SET_COMPARE(tim_inst, TIM_CHANNEL_3, cmd[2]);
    __HAL_TIM_SET_COMPARE(tim_inst, TIM_CHANNEL_4, cmd[3]);

    if (tim_started == 0) {
        tim_started = 1;
        HAL_TIM_PWM_Start(tim_inst, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(tim_inst, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(tim_inst, TIM_CHANNEL_3);
        HAL_TIM_PWM_Start(tim_inst, TIM_CHANNEL_4);
    }

    cmd[0] = calc_compare(read_uint16(input, input_offset));
    cmd[1] = calc_compare(read_uint16(input, input_offset));
    cmd[2] = calc_compare(read_uint16(input, input_offset));
    cmd[3] = calc_compare(read_uint16(input, input_offset));
}

void App_Vanilla_PWM::exit() {
    HAL_TIM_PWM_Stop(tim_inst, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(tim_inst, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(tim_inst, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(tim_inst, TIM_CHANNEL_4);

    if (tim_id == 2) {
        deinit_tim2();
    } else if (tim_id == 2) {
        deinit_tim3();
    }
}

uint32_t App_Vanilla_PWM::calc_compare(uint16_t expected_high_pulse) const {
    return (uint32_t)((((double) expected_high_pulse) / ((double) this->expected_period)) * this->best_arr + 0.5);
}