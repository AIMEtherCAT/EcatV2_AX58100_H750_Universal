//
// Created by Hang XU on 21/10/2025.
//

#ifndef ECATV2_AX58100_H750_UNIVERSAL_BUFFER_MANAGER_H
#define ECATV2_AX58100_H750_UNIVERSAL_BUFFER_MANAGER_H

#include <cstring>

#include "cstdint"
#include "cmsis_os.h"
#include "io_utils.hpp"

#if defined( __ICCARM__ )
#define DMA_BUFFER \
_Pragma("location=\".dma_buffer\"")
#else
#define DMA_BUFFER \
__attribute__((section(".dma_buffer")))
#endif

namespace aim::io::buffer {
    class Buffer final {
    public:
        explicit Buffer(void *buf, const int buf_length) : _buf(buf), _index(0), _buf_length(buf_length) {
            configASSERT(buf != nullptr);
        }

        ~Buffer() = default;

        float read_float16() {
            return little_endian::read_float16(get_buf_pointer<uint8_t>(), &_index);
        }

        float read_float() {
            return little_endian::read_float(get_buf_pointer<uint8_t>(), &_index);
        }

        uint32_t read_uint32() {
            return little_endian::read_uint32(get_buf_pointer<uint8_t>(), &_index);
        }

        int32_t read_int32() {
            return little_endian::read_int32(get_buf_pointer<uint8_t>(), &_index);
        }

        uint16_t read_uint16() {
            return little_endian::read_uint16(get_buf_pointer<uint8_t>(), &_index);
        }

        int16_t read_int16() {
            return little_endian::read_int16(get_buf_pointer<uint8_t>(), &_index);
        }

        uint8_t read_uint8() {
            return little_endian::read_uint8(get_buf_pointer<uint8_t>(), &_index);
        }

        void write_uint8(const uint8_t value) {
            little_endian::write_uint8(value, get_buf_pointer<uint8_t>(), &_index);
        }

        void write_uint16(const uint16_t value) {
            little_endian::write_uint16(value, get_buf_pointer<uint8_t>(), &_index);
        }

        void write_int16(const int16_t value) {
            little_endian::write_int16(value, get_buf_pointer<uint8_t>(), &_index);
        }

        void write_int32(const int32_t value) {
            little_endian::write_int32(value, get_buf_pointer<uint8_t>(), &_index);
        }

        void write_float(const float value) {
            little_endian::write_float(value, get_buf_pointer<uint8_t>(), &_index);
        }

        void read(uint8_t *dst, const int length) {
            memcpy(dst, get_buf_pointer<uint8_t>() + _index, length);
            _index += length;
        }

        void write(const uint8_t *src, const int length) {
            memcpy(get_buf_pointer<uint8_t>() + _index, src, length);
            _index += length;
        }

        void raw_read(uint8_t *dst, const int length) const {
            memcpy(dst, get_buf_pointer<uint8_t>(), length);
        }

        void raw_write(const uint8_t *src, const int length) const {
            memcpy(get_buf_pointer<uint8_t>(), src, length);
        }

        void reset_index() {
            _index = 0;
        }

        void reset() {
            memset(_buf, 0, _buf_length);
            reset_index();
        }

        void skip(const int length) {
            this->_index += length;
        }

        template<typename T>
        [[nodiscard]] T *get_buf_pointer() const {
            return static_cast<T *>(_buf);
        }

    private:
        void *_buf;
        int _index;
        int _buf_length;
    };

    enum class Type {
        DSHOT1_MOTOR1,
        DSHOT1_MOTOR2,
        DSHOT1_MOTOR3,
        DSHOT1_MOTOR4,
        DSHOT2_MOTOR1,
        DSHOT2_MOTOR2,
        DSHOT2_MOTOR3,
        DSHOT2_MOTOR4,
        I2C3_RECV,
        I2C3_SEND,
        UART8_RECV,
        UART8_SEND,
        UART4_RECV,
        UART4_SEND,
        USART1_RECV,
        USART1_SEND,
        ADC1_RECV,

        ECAT_ARGS,
        ECAT_SLAVE_TO_MASTER,
        ECAT_MASTER_TO_SLAVE
    };

    void init_buffer_manager();

    Buffer *get_buffer(Type type);

    void clear_all_buffers();
}

#endif //ECATV2_AX58100_H750_UNIVERSAL_BUFFER_MANAGER_H
