#include "crc16.hpp"

namespace aim::algorithm::crc16 {
    uint16_t get_CRC16_check_sum(const uint8_t *pch_message, uint32_t dw_length, uint16_t wCRC) {
        if (pch_message == nullptr) {
            return 0xFFFF;
        }
        while (dw_length--) {
            const uint8_t chData = *pch_message++;
            wCRC = wCRC >> 8 ^ wCRC_table[(wCRC ^ static_cast<uint16_t>(chData)) & 0x00ff];
        }
        return wCRC;
    }

    uint32_t verify_CRC16_check_sum(const uint8_t *pch_message, const uint32_t dw_length) {
        uint16_t wExpected = 0;
        if (pch_message == nullptr || dw_length <= 2) {
            return 0;
        }
        wExpected = get_CRC16_check_sum(pch_message, dw_length - 2, CRC16_INIT);
        return (wExpected & 0xff) == pch_message[dw_length - 2] && (wExpected >> 8 & 0xff) == pch_message[
                   dw_length - 1];
    }

    void append_CRC16_check_sum(uint8_t *pch_message, const uint32_t dw_length) {
        uint16_t wCRC = 0;
        if (pch_message == nullptr || dw_length <= 2) {
            return;
        }
        wCRC = get_CRC16_check_sum(pch_message, dw_length - 2, CRC16_INIT);
        pch_message[dw_length - 2] = static_cast<uint8_t>(wCRC & 0x00ff);
        pch_message[dw_length - 1] = static_cast<uint8_t>(wCRC >> 8 & 0x00ff);
    }
}
