#ifndef CRC_H
#define CRC_H

#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

uint16_t get_CRC16_check_sum(const uint8_t *pch_message, uint32_t dw_length, uint16_t wCRC);

uint32_t verify_CRC16_check_sum(const uint8_t *pch_message, uint32_t dw_length);

void append_CRC16_check_sum(uint8_t *pch_message, uint32_t dw_length);

#ifdef __cplusplus
}
#endif

#endif