//
// Created by Hang XU on 07/06/2025.
//

#include "../i2c_processor.h"
#include <cstring>
#include <vector>
#include "../task_defs.h"

extern "C" {
#include "i2c.h"
}

extern std::vector<I2CRunnable *> i2c_list;

uint8_t i2c3_ready = 1;

DMA_BUFFER uint8_t i2c3_buf[512];
DMA_BUFFER uint8_t i2c3_send_buf[512];

void reset_i2c_ready() {
    i2c3_ready = 1;

    __HAL_DMA_DISABLE(hi2c3.hdmatx);
    __HAL_DMA_DISABLE(hi2c3.hdmarx);

    hi2c3.State = HAL_I2C_STATE_READY;
    hi2c3.Mode = HAL_I2C_MODE_NONE;
    hi2c3.ErrorCode = HAL_I2C_ERROR_NONE;

    HAL_DMA_Abort(hi2c3.hdmatx);
    HAL_DMA_Abort(hi2c3.hdmarx);
}

void init_i2c_buf() {
    memset(i2c3_buf, 0, 512);
    memset(i2c3_send_buf, 0, 512);
}

void call_i2c_recv_dma(I2C_HandleTypeDef *hi2c, uint16_t addr, uint16_t Size) {
    if (hi2c->Instance == I2C3) {
        HAL_I2C_Master_Receive_DMA(&hi2c3, addr, i2c3_buf, Size);
    }
}

void call_i2c_send_by_dma(I2C_HandleTypeDef *hi2c, uint16_t addr, uint8_t *buf, uint16_t Size) {
    if (hi2c->Instance == I2C3) {
        if (i2c3_ready == 0) {
            return;
        }
        memcpy(i2c3_send_buf, buf, Size);
        HAL_I2C_Master_Transmit_DMA(&hi2c3, addr, i2c3_send_buf, Size);
        i2c3_ready = 0;
    }
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C3) {
        i2c3_ready = 1;
    }
    for (I2CRunnable *runnable: i2c_list) {
        if (hi2c->Instance != runnable->i2c_inst->Instance) {
            continue;
        }

        if (hi2c->Instance == I2C3) {
            runnable->i2c_dma_tx_finished_callback();
        }
    }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    for (I2CRunnable *runnable: i2c_list) {
        if (hi2c->Instance != runnable->i2c_inst->Instance) {
            continue;
        }

        if (hi2c->Instance == I2C3) {
            runnable->i2c_recv(i2c3_buf);
        }
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    for (I2CRunnable *runnable: i2c_list) {
        if (hi2c->Instance != runnable->i2c_inst->Instance) {
            continue;
        }

        runnable->i2c_err();
    }
}