//
// Created by Hang XU on 22/10/2025.
//

#ifndef ECATV2_AX58100_H750_UNIVERSAL_THREAD_SAFE_UTILS_H
#define ECATV2_AX58100_H750_UNIVERSAL_THREAD_SAFE_UTILS_H

#include <atomic>

extern "C" {
#include "stm32h7xx.h"
}

namespace aim::utils::thread_safety {
    class ThreadSafeFlag {
    public:
        ThreadSafeFlag() = default;

        explicit ThreadSafeFlag(const bool cond) {
            if (cond) {
                set();
            }
        }

        void set(const uint8_t value) {
            flag_.store(value, std::memory_order_release);
        }

        void set() {
            flag_.store(true, std::memory_order_release);
        }

        void clear() {
            flag_.store(false, std::memory_order_release);
        }

        [[nodiscard]] bool get() const { return flag_.load(std::memory_order_acquire); }

    private:
        std::atomic<bool> flag_{false};
    };

    class ThreadSafeTimestamp {
    public:
        ThreadSafeTimestamp() = default;

        void set(const uint32_t value) {
            timestamp_.store(value, std::memory_order_release);
        }

        void set_current() {
            set(HAL_GetTick());
        }

        [[nodiscard]] uint32_t get() const { return timestamp_.load(std::memory_order_acquire); }

    private:
        std::atomic<uint32_t> timestamp_{};
    };

    template<typename T>
    class ThreadSafeValue {
    public:
        ThreadSafeValue() = default;

        explicit ThreadSafeValue(const T &value) {
            set(value);
        }

        void set(const T &value) {
            value_.store(value, std::memory_order_release);
        }

        [[nodiscard]] T get() const {
            return value_.load(std::memory_order_acquire);
        }

        void reset() {
            set(T{});
        }

    private:
        std::atomic<T> value_{};
    };

    class ThreadSafeCounter {
    public:
        explicit ThreadSafeCounter(const uint32_t init = 0) : value_(init) {
        }

        void increment() {
            value_.fetch_add(1, std::memory_order_relaxed);
        }

        void decrement() {
            value_.fetch_sub(1, std::memory_order_relaxed);
        }

        [[nodiscard]] uint32_t get() const {
            return value_.load(std::memory_order_relaxed);
        }

        void set(const uint32_t v) {
            value_.store(v, std::memory_order_relaxed);
        }

        void reset() {
            set(0);
        }

    private:
        std::atomic<uint32_t> value_;
    };

    class ThreadSafeBuffer {
    public:
        explicit ThreadSafeBuffer(const uint32_t size)
            : size_(size) {
            buf1_ = new uint8_t[size_];
            buf2_ = new uint8_t[size_];
            active_buf_.store(buf1_, std::memory_order_release);
        }

        ~ThreadSafeBuffer() {
            delete buf1_;
            delete buf2_;
        }

        ThreadSafeBuffer(const ThreadSafeBuffer &) = delete;

        ThreadSafeBuffer &operator=(const ThreadSafeBuffer &) = delete;

        void write(const uint8_t *src, uint8_t length) {
            if (length > size_) {
                length = size_;
            }

            uint8_t *write_buf = active_buf_.load(std::memory_order_acquire) == buf1_ ? buf2_ : buf1_;
            std::memcpy(write_buf, src, length);

            active_buf_.store(write_buf, std::memory_order_release);
        }


        void read(uint8_t *dst, uint8_t length) const {
            if (length > size_) {
                length = size_;
            }
            const uint8_t *read_buf = active_buf_.load(std::memory_order_acquire);
            std::memcpy(dst, read_buf, length);
        }

    private:
        uint32_t size_;
        uint8_t *buf1_;
        uint8_t *buf2_;
        std::atomic<uint8_t *> active_buf_;
    };
}

#endif //ECATV2_AX58100_H750_UNIVERSAL_THREAD_SAFE_UTILS_H
