//
// Created by Hang XU on 21/10/2025.
//

#ifndef ECATV2_AX58100_H750_UNIVERSAL_BUFFER_MANAGER_H
#define ECATV2_AX58100_H750_UNIVERSAL_BUFFER_MANAGER_H

#include "cstring"
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
    enum class EndianType {
        LITTLE,
        BIG,
    };

    class Buffer final {
    public:
        explicit Buffer(void *buf, const int buf_length) : buf_(buf), index_(0), buf_length_(buf_length) {
            configASSERT(buf != nullptr);
        }

        ~Buffer() = default;

        float read_float16(const EndianType endian) {
            return endian == EndianType::BIG
                       ? big_endian::read_float16(get_buf_pointer<uint8_t>(), &index_)
                       : little_endian::read_float16(get_buf_pointer<uint8_t>(), &index_);
        }

        float read_float(const EndianType endian) {
            return endian == EndianType::BIG
                       ? big_endian::read_float(get_buf_pointer<uint8_t>(), &index_)
                       : little_endian::read_float(get_buf_pointer<uint8_t>(), &index_);
        }

        uint32_t read_uint32(const EndianType endian) {
            return endian == EndianType::BIG
                       ? big_endian::read_uint32(get_buf_pointer<uint8_t>(), &index_)
                       : little_endian::read_uint32(get_buf_pointer<uint8_t>(), &index_);
        }

        int32_t read_int32(const EndianType endian) {
            return endian == EndianType::BIG
                       ? big_endian::read_int32(get_buf_pointer<uint8_t>(), &index_)
                       : little_endian::read_int32(get_buf_pointer<uint8_t>(), &index_);
        }

        uint16_t read_uint16(const EndianType endian) {
            return endian == EndianType::BIG
                       ? big_endian::read_uint16(get_buf_pointer<uint8_t>(), &index_)
                       : little_endian::read_uint16(get_buf_pointer<uint8_t>(), &index_);
        }

        int16_t read_int16(const EndianType endian) {
            return endian == EndianType::BIG
                       ? big_endian::read_int16(get_buf_pointer<uint8_t>(), &index_)
                       : little_endian::read_int16(get_buf_pointer<uint8_t>(), &index_);
        }

        uint8_t read_uint8(const EndianType endian) {
            return endian == EndianType::BIG
                       ? big_endian::read_uint8(get_buf_pointer<uint8_t>(), &index_)
                       : little_endian::read_uint8(get_buf_pointer<uint8_t>(), &index_);
        }

        void write_uint8(const EndianType endian, const uint8_t value) {
            return endian == EndianType::BIG
                       ? big_endian::write_uint8(value, get_buf_pointer<uint8_t>(), &index_)
                       : little_endian::write_uint8(value, get_buf_pointer<uint8_t>(), &index_);
        }

        void write_uint16(const EndianType endian, const uint16_t value) {
            return endian == EndianType::BIG
                       ? big_endian::write_uint16(value, get_buf_pointer<uint8_t>(), &index_)
                       : little_endian::write_uint16(value, get_buf_pointer<uint8_t>(), &index_);
        }

        void write_int16(const EndianType endian, const int16_t value) {
            return endian == EndianType::BIG
                       ? big_endian::write_int16(value, get_buf_pointer<uint8_t>(), &index_)
                       : little_endian::write_int16(value, get_buf_pointer<uint8_t>(), &index_);
        }

        void write_int32(const EndianType endian, const int32_t value) {
            return endian == EndianType::BIG
                       ? big_endian::write_int32(value, get_buf_pointer<uint8_t>(), &index_)
                       : little_endian::write_int32(value, get_buf_pointer<uint8_t>(), &index_);
        }

        void write_float(const EndianType endian, const float value) {
            return endian == EndianType::BIG
                       ? big_endian::write_float(value, get_buf_pointer<uint8_t>(), &index_)
                       : little_endian::write_float(value, get_buf_pointer<uint8_t>(), &index_);
        }

        void read(uint8_t *dst, const int length) {
            memcpy(dst, get_buf_pointer<uint8_t>() + index_, length);
            index_ += length;
        }

        void write(const uint8_t *src, const int length) {
            memcpy(get_buf_pointer<uint8_t>() + index_, src, length);
            index_ += length;
        }

        void raw_read(uint8_t *dst, const int length) const {
            memcpy(dst, get_buf_pointer<uint8_t>(), length);
        }

        void raw_write(const uint8_t *src, const int length) const {
            memcpy(get_buf_pointer<uint8_t>(), src, length);
        }

        void reset_index() {
            index_ = 0;
        }

        void reset() {
            memset(buf_, 0, buf_length_);
            reset_index();
        }

        void skip(const int length) {
            this->index_ += length;
        }

        template<typename T>
        [[nodiscard]] T *get_buf_pointer() const {
            return static_cast<T *>(buf_);
        }

    private:
        void *buf_;
        int index_;
        int buf_length_;
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
