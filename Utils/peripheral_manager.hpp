//
// Created by Hang XU on 04/06/2025.
//

#ifndef DEVICE_CONF_H
#define DEVICE_CONF_H

#include "buffer_manager.hpp"
#include "cmsis_os.h"
#include "thread_safe_utils.hpp"

extern "C" {
#include "usart.h"
#include "fdcan.h"
#include "i2c.h"
#include "tim.h"
#include "adc.h"
#include "stm32h7xx_hal_dma.h"
}

namespace aim::hardware::peripheral {
    using namespace io;

    class Peripheral {
    public:
        explicit Peripheral(const osMutexId mutex) : _mutex(mutex), _inited(false) {
            configASSERT(mutex != nullptr);
        }

        virtual ~Peripheral() = default;

        osStatus init() {
            if (osMutexWait(_mutex, osWaitForever) != osOK) {
                return osErrorResource;
            }

            if (_inited) {
                osMutexRelease(_mutex);
                return osOK;
            }

            _init_impl();
            _inited = true;

            osMutexRelease(_mutex);
            return osOK;
        }

        osStatus deinit() {
            if (osMutexWait(_mutex, osWaitForever) != osOK) {
                return osErrorResource;
            }

            if (!_inited) {
                osMutexRelease(_mutex);
                return osOK;
            }

            _deinit_impl();
            _inited = false;

            osMutexRelease(_mutex);
            return osOK;
        }

        [[nodiscard]] uint8_t isInited() const {
            return _inited;
        }

    protected:
        virtual void _init_impl() {
        }


        virtual void _deinit_impl() {
        }

    private:
        osMutexId _mutex;
        volatile uint8_t _inited;
    };

    class UartPeripheral : public Peripheral {
    public:
        explicit UartPeripheral(const osMutexId mutex, UART_HandleTypeDef *huart, buffer::Buffer *send_buf,
                                buffer::Buffer *recv_buf) : Peripheral(mutex), _huart(huart),
                                                            _send_buf(send_buf),
                                                            _recv_buf(recv_buf) {
        }

        void receive_by_dma(const uint16_t size) const {
            HAL_UARTEx_ReceiveToIdle_DMA(_huart, _recv_buf->get_buf_pointer<uint8_t>(), size);
        }

        void reset_tx_dma() const {
            HAL_DMA_Abort(_huart->hdmatx);

            __HAL_DMA_DISABLE(_huart->hdmatx);
            __HAL_DMA_CLEAR_FLAG(_huart->hdmatx,
                 __HAL_DMA_GET_TC_FLAG_INDEX(_huart->hdmatx) |
                 __HAL_DMA_GET_HT_FLAG_INDEX(_huart->hdmatx) |
                 __HAL_DMA_GET_TE_FLAG_INDEX(_huart->hdmatx)
                );
            __HAL_DMA_ENABLE(_huart->hdmatx);
            _huart->gState = HAL_UART_STATE_READY;
            _huart->RxState = HAL_UART_STATE_READY;
            _huart->TxXferCount = 0;
            _huart->TxXferSize = 0;

            __HAL_UART_CLEAR_FLAG(_huart,
                UART_FLAG_TC |
                UART_FLAG_TXE |
                UART_FLAG_FE |
                UART_FLAG_NE |
                UART_FLAG_ORE
            );
        }

        bool send_by_dma(const uint8_t *buffer, const uint16_t size) {
            if (is_busy.get()) {
                return false;
            }
            _send_buf->raw_write(buffer, size);
            HAL_UART_Transmit_DMA(&huart1, _send_buf->get_buf_pointer<uint8_t>(), size);
            _send_buf->reset_index();
            is_busy.set();
            return true;
        }

        utils::thread_safety::ThreadSafeFlag is_busy;
        UART_HandleTypeDef *_huart;
        buffer::Buffer *_send_buf;
        buffer::Buffer *_recv_buf;
    };


    class USART1Peripheral final : public UartPeripheral {
    public:
        explicit USART1Peripheral(const osMutexId mutex, UART_HandleTypeDef *huart, buffer::Buffer *send_buf,
                                  buffer::Buffer *recv_buf) : UartPeripheral(
            mutex, huart, send_buf, recv_buf) {
        }

    protected:
        void _init_impl() override;

        void _deinit_impl() override;
    };

    class CANPeripheral final : public Peripheral {
    public:
        explicit CANPeripheral(const osMutexId mutex) : Peripheral(mutex) {
        }

    protected:
        void _init_impl() override;

        void _deinit_impl() override;
    };

    class UART4Peripheral final : public UartPeripheral {
    public:
        explicit UART4Peripheral(const osMutexId mutex, UART_HandleTypeDef *huart, buffer::Buffer *send_buf,
                                 buffer::Buffer *recv_buf) : UartPeripheral(
            mutex, huart, send_buf, recv_buf) {
        }

    protected:
        void _init_impl() override;

        void _deinit_impl() override;
    };

    class UART8Peripheral final : public UartPeripheral {
    public:
        explicit UART8Peripheral(const osMutexId mutex, UART_HandleTypeDef *huart, buffer::Buffer *send_buf,
                                 buffer::Buffer *recv_buf) : UartPeripheral(
            mutex, huart, send_buf, recv_buf) {
        }

    protected:
        void _init_impl() override;

        void _deinit_impl() override;
    };

    class TIM2Peripheral final : public Peripheral {
    public:
        explicit TIM2Peripheral(const osMutexId mutex) : Peripheral(mutex) {
        }

    protected:
        void _init_impl() override;

        void _deinit_impl() override;
    };

    class TIM3Peripheral final : public Peripheral {
    public:
        explicit TIM3Peripheral(const osMutexId mutex) : Peripheral(mutex) {
        }

    protected:
        void _init_impl() override;

        void _deinit_impl() override;
    };

    class I2C3Peripheral final : public Peripheral {
    public:
        explicit I2C3Peripheral(const osMutexId mutex) : Peripheral(mutex) {
        }

    protected:
        void _init_impl() override;

        void _deinit_impl() override;
    };

    class ADC1Peripheral final : public Peripheral {
    public:
        explicit ADC1Peripheral(const osMutexId mutex) : Peripheral(mutex) {
        }

    protected:
        void _init_impl() override;

        void _deinit_impl() override;
    };

    enum class Type {
        PERIPHERAL_USART1,
        PERIPHERAL_UART4,
        PERIPHERAL_UART8,
        PERIPHERAL_I2C3,
        PERIPHERAL_CAN,
        PERIPHERAL_TIM2,
        PERIPHERAL_TIM3,
        PERIPHERAL_ADC1
    };

    Peripheral *get_peripheral(Type type);

    void init_peripheral_manager();
}

#endif //DEVICE_CONF_H
