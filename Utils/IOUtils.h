//
// Created by Hang XU on 21/04/2025.
//

#ifndef IOUTILS_H
#define IOUTILS_H

#include "stm32h7xx.h"
#ifdef __cplusplus
extern "C" {
#endif
float read_float16(uint8_t *t, int *offset);

float read_float(uint8_t *t, int *offset);

uint8_t read_uint8(uint8_t *t, int *offset);

int16_t read_int16(uint8_t *t, int *offset);

uint16_t read_uint16(uint8_t *t, int *offset);

uint32_t read_uint32(uint8_t *t, int *offset);

void write_uint8(uint8_t value, uint8_t *t, int *offset);

void write_float(float value, uint8_t *t, int *offset);

void write_int16(int16_t value, uint8_t *t, int *offset);

void write_int32(int32_t value, uint8_t *t, int *offset);

void write_uint16(uint16_t value, uint8_t *t, int *offset);

int32_t read_int32(uint8_t *t, int *offset);
#ifdef __cplusplus
}
#endif
#endif //IOUTILS_H
