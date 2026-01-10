//
// Created by Hang XU on 04/06/2025.
//

#ifndef DEVICE_CONF_H
#define DEVICE_CONF_H

#include "cmsis_os.h"
#include "buffer_utils.hpp"
#include "thread_safe_utils.hpp"

extern "C" {
#include "stm32h7xx.h"
}

namespace aim::hardware::peripheral {
    using namespace io;

    class Peripheral {
    public:
        explicit Peripheral(const osMutexId mutex) : mutex_(mutex), inited_(false) {
            configASSERT(mutex != nullptr);
        }

        virtual ~Peripheral() = default;

        osStatus init() {
            if (osMutexWait(mutex_, osWaitForever) != osOK) {
                return osErrorResource;
            }

            if (inited_) {
                osMutexRelease(mutex_);
                return osOK;
            }

            _init_impl();
            inited_ = true;

            osMutexRelease(mutex_);
            return osOK;
        }

        osStatus deinit() {
            if (osMutexWait(mutex_, osWaitForever) != osOK) {
                return osErrorResource;
            }

            if (!inited_) {
                osMutexRelease(mutex_);
                return osOK;
            }

            _deinit_impl();
            inited_ = false;

            osMutexRelease(mutex_);
            return osOK;
        }

        [[nodiscard]] uint8_t isInited() const {
            return inited_;
        }

    protected:
        virtual void _init_impl() {
        }


        virtual void _deinit_impl() {
        }

    private:
        osMutexId mutex_;
        volatile uint8_t inited_;
    };

    class UartPeripheral : public Peripheral {
    public:
        explicit UartPeripheral(const osMutexId mutex, UART_HandleTypeDef *huart, buffer::Buffer *send_buf,
                                buffer::Buffer *recv_buf) : Peripheral(mutex), huart_(huart),
                                                            send_buf_(send_buf),
                                                            recv_buf_(recv_buf) {
        }

        void receive_by_dma(const uint16_t size) const {
            HAL_UARTEx_ReceiveToIdle_DMA(huart_, recv_buf_->get_buf_pointer<uint8_t>(), size);
        }

        void reset_tx_dma() const {
            HAL_DMA_Abort(huart_->hdmatx);

            __HAL_DMA_DISABLE(huart_->hdmatx);
            __HAL_DMA_CLEAR_FLAG(huart_->hdmatx,
                                 __HAL_DMA_GET_TC_FLAG_INDEX(huart_->hdmatx) |
                                 __HAL_DMA_GET_HT_FLAG_INDEX(huart_->hdmatx) |
                                 __HAL_DMA_GET_TE_FLAG_INDEX(huart_->hdmatx)
            );
            __HAL_DMA_ENABLE(huart_->hdmatx);
            huart_->gState = HAL_UART_STATE_READY;
            huart_->RxState = HAL_UART_STATE_READY;
            huart_->TxXferCount = 0;
            huart_->TxXferSize = 0;

            __HAL_UART_CLEAR_FLAG(huart_,
                                  UART_FLAG_TC |
                                  UART_FLAG_TXE |
                                  UART_FLAG_FE |
                                  UART_FLAG_NE |
                                  UART_FLAG_ORE
            );
        }

        bool send_by_dma(const uint8_t *buffer, const uint16_t size) {
            if (is_busy_.get()) {
                return false;
            }
            send_buf_->raw_write(buffer, size);
            if (HAL_UART_Transmit_DMA(huart_, send_buf_->get_buf_pointer<uint8_t>(), size) != HAL_OK) {
                return false;
            }
            send_buf_->reset_index();
            is_busy_.set();
            return true;
        }

        utils::thread_safety::ThreadSafeFlag is_busy_;
        UART_HandleTypeDef *huart_;
        buffer::Buffer *send_buf_;
        buffer::Buffer *recv_buf_;
    };

    class I2CPeripheral : public Peripheral {
    public:
        explicit I2CPeripheral(const osMutexId mutex, I2C_HandleTypeDef *hi2c,
                               buffer::Buffer *send_buf, buffer::Buffer *recv_buf)
            : Peripheral(mutex),
              hi2c_(hi2c),
              send_buf_(send_buf),
              recv_buf_(recv_buf) {
        }

        void receive_by_dma(const uint16_t dev_addr, const uint16_t size) const {
            HAL_I2C_Master_Receive_DMA(hi2c_, dev_addr, recv_buf_->get_buf_pointer<uint8_t>(), size);
        }

        bool send_by_dma(const uint16_t dev_addr, const uint8_t *buffer, const uint16_t size) {
            if (is_busy_.get()) {
                return false;
            }
            send_buf_->raw_write(buffer, size);
            if (HAL_I2C_Master_Transmit_DMA(hi2c_, dev_addr, send_buf_->get_buf_pointer<uint8_t>(), size) != HAL_OK) {
                return false;
            }
            send_buf_->reset_index();
            is_busy_.set();
            return true;
        }

        void reset_tx_dma() const {
            HAL_DMA_Abort(hi2c_->hdmatx);

            __HAL_DMA_DISABLE(hi2c_->hdmatx);

            __HAL_DMA_CLEAR_FLAG(hi2c_->hdmatx,
                                 __HAL_DMA_GET_TC_FLAG_INDEX(hi2c_->hdmatx) |
                                 __HAL_DMA_GET_HT_FLAG_INDEX(hi2c_->hdmatx) |
                                 __HAL_DMA_GET_TE_FLAG_INDEX(hi2c_->hdmatx)
            );

            __HAL_DMA_ENABLE(hi2c_->hdmatx);

            hi2c_->State = HAL_I2C_STATE_READY;
            hi2c_->PreviousState = HAL_I2C_STATE_READY;
            hi2c_->ErrorCode = HAL_I2C_ERROR_NONE;
            hi2c_->XferSize = 0;
            hi2c_->XferCount = 0;

            __HAL_I2C_CLEAR_FLAG(hi2c_,
                                 I2C_FLAG_TXE |
                                 I2C_FLAG_BERR
            );
        }

        utils::thread_safety::ThreadSafeFlag is_busy_;
        I2C_HandleTypeDef *hi2c_;
        buffer::Buffer *send_buf_;
        buffer::Buffer *recv_buf_;
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

    class I2C3Peripheral final : public I2CPeripheral {
    public:
        explicit I2C3Peripheral(const osMutexId mutex, I2C_HandleTypeDef *hi2c,
                                buffer::Buffer *send_buf, buffer::Buffer *recv_buf) : I2CPeripheral(
            mutex, hi2c, send_buf, recv_buf) {
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
