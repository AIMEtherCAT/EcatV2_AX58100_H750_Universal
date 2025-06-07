#ifndef CRC8_CRC16_H
#define CRC8_CRC16_H

#include "main.h"
#ifdef __cplusplus
extern "C" {
#endif
/**
  * @brief          calculate the crc8  
  * @param[in]      pch_message: data
  * @param[in]      dw_length: stream length = data + checksum
  * @param[in]      ucCRC8: init CRC8
  * @retval         calculated crc8
  */
extern uint8_t get_CRC8_check_sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8);

/**
  * @brief          CRC8 verify function  
  * @param[in]      pch_message: data
  * @param[in]      dw_length:stream length = data + checksum
  * @retval         true of false
  */
extern uint32_t verify_CRC8_check_sum(unsigned char *pchMessage, unsigned int dwLength);

/**
  * @brief          append CRC8 to the end of data
  * @param[in]      pch_message: data
  * @param[in]      dw_length:stream length = data + checksum
  * @retval         none
  */
extern void append_CRC8_check_sum(unsigned char *pchMessage, unsigned int dwLength);

/**
  * @brief          calculate the crc16  
  * @param[in]      pch_message: data
  * @param[in]      dw_length: stream length = data + checksum
  * @param[in]      wCRC: init CRC16
  * @retval         calculated crc16
  */
extern uint16_t get_CRC16_check_sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);

/**
  * @brief          CRC16 verify function  
  * @param[in]      pch_message: data
  * @param[in]      dw_length:stream length = data + checksum
  * @retval         true of false
  */
extern uint32_t verify_CRC16_check_sum(uint8_t *pchMessage, uint32_t dwLength);

/**
  * @brief          append CRC16 to the end of data
  * @param[in]      pch_message: data
  * @param[in]      dw_length:stream length = data + checksum
  * @retval         none
  */
extern void append_CRC16_check_sum(uint8_t *pchMessage, uint32_t dwLength);
#ifdef __cplusplus
  }
#endif
#endif
