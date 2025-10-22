//
// Created by Hang XU on 07/06/2025.
//
#include "io_utils.h"
#include "../task_defs.h"
#include "../task_manager.h"
#include "../i2c_processor.h"

extern "C" {
#include "i2c.h"
#include "peripheral_conf.hpp"
}

App_MS5837_30BA::App_MS5837_30BA(uint8_t *args, int *offset) {
    current_stage = INITIALIZING;
    i2c_id = read_uint8(args, offset);
    if (i2c_id == 3) {
        i2c_inst = &hi2c3;
        init_i2c3();
    }
    osr_id = read_uint8(args, offset);

    switch (osr_id) {
        case 1: {
            d1_cmd = MS5837_D1_OSR256_CMD;
            d2_cmd = MS5837_D2_OSR256_CMD;
            adc_wait_time = 1;
            break;
        }
        case 2: {
            d1_cmd = MS5837_D1_OSR512_CMD;
            d2_cmd = MS5837_D2_OSR512_CMD;
            adc_wait_time = 2;
            break;
        }
        case 3: {
            d1_cmd = MS5837_D1_OSR1024_CMD;
            d2_cmd = MS5837_D2_OSR1024_CMD;
            adc_wait_time = 3;
            break;
        }
        case 4: {
            d1_cmd = MS5837_D1_OSR2048_CMD;
            d2_cmd = MS5837_D2_OSR2048_CMD;
            adc_wait_time = 5;
            break;
        }
        case 5: {
            d1_cmd = MS5837_D1_OSR4096_CMD;
            d2_cmd = MS5837_D2_OSR4096_CMD;
            adc_wait_time = 10;
            break;
        }
        case 6: {
            d1_cmd = MS5837_D1_OSR8192_CMD;
            d2_cmd = MS5837_D2_OSR8192_CMD;
            adc_wait_time = 19;
            break;
        }
        default: {
        }
    }
    send_called = 0;
    recv_called = 0;
}

void App_MS5837_30BA::collect_inputs(uint8_t *input, int *input_offset) {
    switch (current_stage) {
        case INITIALIZING: {
            if (send_called == 0) {
                buf[0] = MS5837_RESET_CMD;
                call_i2c_send_by_dma(i2c_inst, MS5837_ADDR, buf, 1);
                send_called = 1;
                last_act_ts = HAL_GetTick();
            }
            if (send_called == 1) {
                if (HAL_GetTick() - last_act_ts >= 100) {
                    send_called = 0;
                    recv_called = 0;
                    current_stage = READING_C1;
                }
            }
            break;
        }
        case READING_C1: {
            if (send_called == 0) {
                buf[0] = MS5837_PROM_READ_CMD_BEGIN + (1 * 2);
                call_i2c_send_by_dma(i2c_inst, MS5837_ADDR, buf, 1);
                tx_finished = 0;
                send_called = 1;
            }
            if (tx_finished == 1) {
                call_i2c_recv_dma(i2c_inst, MS5837_ADDR, 2);
                tx_finished = 0;
            }
            break;
        }
        case READING_C2: {
            if (send_called == 0) {
                buf[0] = MS5837_PROM_READ_CMD_BEGIN + (2 * 2);
                call_i2c_send_by_dma(i2c_inst, MS5837_ADDR, buf, 1);
                tx_finished = 0;
                send_called = 1;
            }
            if (tx_finished == 1) {
                call_i2c_recv_dma(i2c_inst, MS5837_ADDR, 2);
                tx_finished = 0;
            }
            break;
        }
        case READING_C3: {
            if (send_called == 0) {
                buf[0] = MS5837_PROM_READ_CMD_BEGIN + (3 * 2);
                call_i2c_send_by_dma(i2c_inst, MS5837_ADDR, buf, 1);
                tx_finished = 0;
                send_called = 1;
            }
            if (tx_finished == 1) {
                call_i2c_recv_dma(i2c_inst, MS5837_ADDR, 2);
                tx_finished = 0;
            }
            break;
        }
        case READING_C4: {
            if (send_called == 0) {
                buf[0] = MS5837_PROM_READ_CMD_BEGIN + (4 * 2);
                call_i2c_send_by_dma(i2c_inst, MS5837_ADDR, buf, 1);
                tx_finished = 0;
                send_called = 1;
            }
            if (tx_finished == 1) {
                call_i2c_recv_dma(i2c_inst, MS5837_ADDR, 2);
                tx_finished = 0;
            }
            break;
        }
        case READING_C5: {
            if (send_called == 0) {
                buf[0] = MS5837_PROM_READ_CMD_BEGIN + (5 * 2);
                call_i2c_send_by_dma(i2c_inst, MS5837_ADDR, buf, 1);
                tx_finished = 0;
                send_called = 1;
            }
            if (tx_finished == 1) {
                call_i2c_recv_dma(i2c_inst, MS5837_ADDR, 2);
                tx_finished = 0;
            }
            break;
        }
        case READING_C6: {
            if (send_called == 0) {
                buf[0] = MS5837_PROM_READ_CMD_BEGIN + (6 * 2);
                call_i2c_send_by_dma(i2c_inst, MS5837_ADDR, buf, 1);
                tx_finished = 0;
                send_called = 1;
            }
            if (tx_finished == 1) {
                call_i2c_recv_dma(i2c_inst, MS5837_ADDR, 2);
                tx_finished = 0;
            }
            break;
        }
        case WAITING: {
            if (HAL_GetTick() - last_act_ts > 1) {
                recv_called = 0;
                send_called = 0;
                current_stage = READING_D1;
            }
            break;
        }
        case READING_D1: {
            if (send_called == 0) {
                buf[0] = d1_cmd;
                call_i2c_send_by_dma(i2c_inst, MS5837_ADDR, buf, 1);
                send_called = 1;
                last_act_ts = HAL_GetTick();
            }
            if (send_called == 1 && recv_called == 0) {
                if (HAL_GetTick() - last_act_ts > adc_wait_time) {
                    buf[0] = MS5837_ADC_READ_CMD;
                    call_i2c_send_by_dma(i2c_inst, MS5837_ADDR, buf, 1);
                    recv_called = 1;
                    tx_finished = 0;
                }
            }
            if (tx_finished == 1 && recv_called == 1) {
                call_i2c_recv_dma(i2c_inst, MS5837_ADDR, 3);
            }
            break;
        }
        case READING_D2: {
            if (send_called == 0) {
                buf[0] = d2_cmd;
                call_i2c_send_by_dma(i2c_inst, MS5837_ADDR, buf, 1);
                send_called = 1;
                last_act_ts = HAL_GetTick();
            }
            if (send_called == 1 && recv_called == 0) {
                if (HAL_GetTick() - last_act_ts > adc_wait_time) {
                    buf[0] = MS5837_ADC_READ_CMD;
                    call_i2c_send_by_dma(i2c_inst, MS5837_ADDR, buf, 1);
                    recv_called = 1;
                    tx_finished = 0;
                }
            }
            if (tx_finished == 1 && recv_called == 1) {
                call_i2c_recv_dma(i2c_inst, MS5837_ADDR, 3);
            }
            break;
        }
    }
}

void App_MS5837_30BA::i2c_recv(uint8_t *rx_data) {
    switch (current_stage) {
        case READING_C1: {
            C1_Pressure_sensitivity = (rx_data[0] << 8) | rx_data[1];
            current_stage = READING_C2;
            send_called = 0;
            recv_called = 0;
            break;
        }
        case READING_C2: {
            C2_Pressure_offset = (rx_data[0] << 8) | rx_data[1];
            current_stage = READING_C3;
            send_called = 0;
            break;
        }
        case READING_C3: {
            C3_Temperature_coefficient_of_pressure_sensitivity = (rx_data[0] << 8) | rx_data[1];
            current_stage = READING_C4;
            send_called = 0;
            break;
        }
        case READING_C4: {
            C4_Temperature_coefficient_of_pressure_offset = (rx_data[0] << 8) | rx_data[1];
            current_stage = READING_C5;
            send_called = 0;
            break;
        }
        case READING_C5: {
            C5_Reference_temperature = (rx_data[0] << 8) | rx_data[1];
            current_stage = READING_C6;
            send_called = 0;
            break;
        }
        case READING_C6: {
            C6_Temperature_coefficient_of_the_temperature = (rx_data[0] << 8) | rx_data[1];
            current_stage = WAITING;
            send_called = 0;
            last_act_ts = HAL_GetTick();
            inited = 1;
            last_recv_time = HAL_GetTick();
            break;
        }
        case READING_D1: {
            D1_Digital_pressure_value = (rx_data[0] << 16) | (rx_data[1] << 8) | rx_data[2];
            current_stage = READING_D2;
            send_called = 0;
            recv_called = 0;
            break;
        }
        case READING_D2: {
            D2_Digital_temperature_value = (rx_data[0] << 16) | (rx_data[1] << 8) | rx_data[2];
            current_stage = WAITING;
            send_called = 0;
            recv_called = 0;
            last_act_ts = HAL_GetTick();

            dT = (int32_t) D2_Digital_temperature_value - (int32_t) C5_Reference_temperature * 256;
            TEMP = (int64_t) 2000 + (int64_t) dT * (int64_t) C6_Temperature_coefficient_of_the_temperature / (int64_t)
                   8388608;
            OFF = (int64_t) C2_Pressure_offset * 65536 + (
                      (int64_t) C4_Temperature_coefficient_of_pressure_offset * (int64_t) dT) / (int64_t) 128;
            SENS = (int64_t) C1_Pressure_sensitivity * 32768 + (
                       (int64_t) C3_Temperature_coefficient_of_pressure_sensitivity * (int64_t) dT) / (int64_t) 256;
            P = (int64_t) ((int64_t) D1_Digital_pressure_value * (int64_t) SENS / (int64_t) 2097152 - (int64_t) OFF) / (
                    int64_t) 8192;

            if (TEMP / 100 < 20) {
                Ti = (int64_t) 3 * (int64_t) dT * (int64_t) dT / (int64_t) 8589934592;
                OFFi = (int64_t) 3 * ((int64_t) TEMP - (int64_t) 2000) * ((int64_t) TEMP - (int64_t) 2000) / (int64_t)
                       2;
                SENSi = (int64_t) 5 * ((int64_t) TEMP - (int64_t) 2000) * ((int64_t) TEMP - (int64_t) 2000) / (int64_t)
                        8;

                if (TEMP / 100 < -15) {
                    OFFi = OFFi + (int64_t) 7 * ((int64_t) TEMP + (int64_t) 1500) * ((int64_t) TEMP + (int64_t) 1500);
                    SENSi = SENSi + (int64_t) 4 * ((int64_t) TEMP + (int64_t) 1500) * ((int64_t) TEMP + (int64_t) 1500);
                }
            } else {
                Ti = (int64_t) 2 * (int64_t) dT * (int64_t) dT / (int64_t) 137438953472;
                OFFi = ((int64_t) TEMP - (int64_t) 2000) * ((int64_t) TEMP - (int64_t) 2000) / 16;
                SENSi = 0;
            }

            OFF2 = OFF - OFFi;
            SENS2 = SENS - SENSi;

            // degc / 100
            _TEMP2 = (TEMP - Ti);
            // mbar / 10
            _P2 = (((D1_Digital_pressure_value * SENS2) / 2097152 - OFF2) / 8192);

            if (_TEMP2 > -30 * 100) {
                TEMP2 = _TEMP2;
            }
            if (_P2 > 800 * 10) {
                P2 = _P2;
            }
            last_recv_time = HAL_GetTick();
            err_times = 0;
            break;
        }
    }
}

void App_MS5837_30BA::collect_outputs(uint8_t *output, int *output_offset) {
    write_int32(TEMP2, output, output_offset);
    write_int32(P2, output, output_offset);

    // 已初始化并且离线时间大于100ms 认为设备离线 进入initializing状态
    if (HAL_GetTick() - last_recv_time > 100 && inited == 1 && err_times >= 100) {
        inited = 0;
        recv_called = 0;
        send_called = 0;
        tx_finished = 0;
        reset_i2c_ready();
        current_stage = INITIALIZING;
        err_times = 0;
        return;
    }

    if (HAL_GetTick() - last_act_ts > 200 && current_stage == READING_C1) {
        inited = 0;
        recv_called = 0;
        send_called = 0;
        tx_finished = 0;
        reset_i2c_ready();
        current_stage = INITIALIZING;
        return;
    }

    // 已初始化并且离线时间大于8倍adc时间 认为接触不良但是设备设备没有掉电
    // 重置i2c后进入waiting状态
    if (HAL_GetTick() - last_recv_time > adc_wait_time * 8 &&
        (current_stage == READING_D1 || current_stage == READING_D2) &&
        inited == 1 &&
        HAL_GetTick() - last_rst_time > adc_wait_time * 8) {
        recv_called = 0;
        send_called = 0;
        tx_finished = 0;
        reset_i2c_ready();
        last_rst_time = HAL_GetTick();
        current_stage = WAITING;
        err_times++;
        return;
    }
}

void App_MS5837_30BA::exit() {
    deinit_i2c3();
}

void App_MS5837_30BA::i2c_dma_tx_finished_callback() {
    tx_finished = 1;
}

void App_MS5837_30BA::i2c_recv_err() {
    switch (current_stage) {
        case READING_C1:
        case READING_C2:
        case READING_C3:
        case READING_C4:
        case READING_C5:
        case READING_C6: {
            call_i2c_recv_dma(i2c_inst, MS5837_ADDR, 2);
            break;
        }
        case READING_D1:
        case READING_D2: {
            call_i2c_recv_dma(i2c_inst, MS5837_ADDR, 3);
            break;
        }
    }
}
