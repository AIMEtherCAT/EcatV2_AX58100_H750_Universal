//
// Created by Hang XU on 21/04/2025.
//

#include "IOUtils.h"

float read_float16(uint8_t *t, int *offset) {
    uint16_t h = read_uint16(t, offset);
    uint16_t h_exp = (h & 0x7C00) >> 10;
    uint16_t h_frac = h & 0x03FF;
    int32_t sign = (h & 0x8000) ? -1 : 1;

    if (h_exp == 0)
    {
        if (h_frac == 0)
            return sign * 0.0f;
        return sign * powf(2, -14) * (h_frac / 1024.0f);
    }
    else if (h_exp == 0x1F)
    {
        return h_frac ? NAN : (sign > 0 ? INFINITY : -INFINITY);
    }
    else
    {
        return sign * powf(2, (int)(h_exp - 15)) * (1.0f + h_frac / 1024.0f);
    }
}

float read_float(uint8_t *t, int *offset) {
    float res = 0.0f;
    uint8_t *res_t = (uint8_t *) &res;
    *(res_t) = *(t + (*offset));
    *(res_t + 1) = *(t + (*offset + 1));
    *(res_t + 2) = *(t + (*offset + 2));
    *(res_t + 3) = *(t + (*offset + 3));
    (*offset) += 4;
    return res;
}

uint32_t read_uint32(uint8_t *t, int *offset) {
    uint32_t res = 0.0f;
    uint8_t *res_t = (uint8_t *) &res;
    *(res_t) = *(t + (*offset));
    *(res_t + 1) = *(t + (*offset + 1));
    *(res_t + 2) = *(t + (*offset + 2));
    *(res_t + 3) = *(t + (*offset + 3));
    (*offset) += 4;
    return res;
}

uint8_t read_uint8(uint8_t *t, int *offset) {
    uint8_t res = *(t + *offset);
    (*offset) += 1;
    return res;
}

int16_t read_int16(uint8_t *t, int *offset) {
    int16_t res = ((int16_t) *(t + *offset + 1) << 8) | *(t + (*offset));
    (*offset) += 2;
    return res;
}

uint16_t read_uint16(uint8_t *t, int *offset) {
    uint16_t res = ((uint16_t) *(t + *offset + 1) << 8) | *(t + (*offset));
    (*offset) += 2;
    return res;
}

void write_uint16(uint16_t value, uint8_t *t, int *offset) {
    *(t + *offset) = value & 0xFF;
    (*offset) += 1;
    *(t + *offset) = value >> 8 & 0xFF;
    (*offset) += 1;
}

void write_int16(int16_t value, uint8_t *t, int *offset) {
    *(t + *offset) = value & 0xFF;
    (*offset) += 1;
    *(t + *offset) = value >> 8 & 0xFF;
    (*offset) += 1;
}

void write_uint8(uint8_t value, uint8_t *t, int *offset) {
    *(t + *offset) = value;
    (*offset) += 1;
}

void write_float(float value, uint8_t *t, int *offset) {
    uint8_t *value_t = (uint8_t *) &value;
    *(t + *offset) = *(value_t + 0);
    (*offset) += 1;
    *(t + *offset) = *(value_t + 1);
    (*offset) += 1;
    *(t + *offset) = *(value_t + 2);
    (*offset) += 1;
    *(t + *offset) = *(value_t + 3);
    (*offset) += 1;
}

void write_int32(int32_t value, uint8_t *t, int *offset) {
    uint8_t *value_t = (uint8_t *) &value;
    *(t + *offset) = *(value_t + 0);
    (*offset) += 1;
    *(t + *offset) = *(value_t + 1);
    (*offset) += 1;
    *(t + *offset) = *(value_t + 2);
    (*offset) += 1;
    *(t + *offset) = *(value_t + 3);
    (*offset) += 1;
}

int32_t read_int32(uint8_t *t, int *offset) {
    int32_t res = 0.0f;
    uint8_t *res_t = (uint8_t *) &res;
    *(res_t) = *(t + (*offset));
    *(res_t + 1) = *(t + (*offset + 1));
    *(res_t + 2) = *(t + (*offset + 2));
    *(res_t + 3) = *(t + (*offset + 3));
    (*offset) += 4;
    return res;
}