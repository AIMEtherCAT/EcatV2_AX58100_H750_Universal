//
// Created by Hang XU on 07/06/2025.
//

#ifndef I2C_PROCESSOR_H
#define I2C_PROCESSOR_H

#include "i2c.h"

void call_i2c_recv_dma(I2C_HandleTypeDef *huart, uint16_t addr, uint16_t Size);

void call_i2c_send_by_dma(I2C_HandleTypeDef *huart, uint16_t addr, uint8_t *buf, uint16_t Size);

void init_i2c_buf();

#endif //I2C_PROCESSOR_H
