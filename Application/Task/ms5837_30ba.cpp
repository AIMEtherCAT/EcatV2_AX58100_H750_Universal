//
// Created by Hang XU on 24/10/2025.
//
#include "buffer_utils.hpp"
#include "peripheral_utils.hpp"
#include "task_defs.hpp"

namespace aim::ecat::task::ms5837 {
    MS5837_30BA::MS5837_30BA(buffer::Buffer *buffer) : I2CRunnable(true) {
        period = 1;

        i2c_dma_tx_sem_ = xSemaphoreCreateBinary();
        configASSERT(i2c_dma_tx_sem_);
        xSemaphoreTake(i2c_dma_tx_sem_, 0);

        i2c_dma_rx_sem_ = xSemaphoreCreateBinary();
        configASSERT(i2c_dma_rx_sem_);
        xSemaphoreTake(i2c_dma_rx_sem_, 0);

        switch (buffer->read_uint8(buffer::EndianType::LITTLE)) {
            case 3: {
                init_peripheral(peripheral::Type::PERIPHERAL_I2C3);
                break;
            }
            default: {
            }
        }

        osr_id_ = buffer->read_uint8(buffer::EndianType::LITTLE);
        switch (osr_id_) {
            case 1: {
                d1_cmd_ = D1_OSR256_CMD;
                d2_cmd_ = D2_OSR256_CMD;
                adc_wait_time_ = 1;
                break;
            }
            case 2: {
                d1_cmd_ = D1_OSR512_CMD;
                d2_cmd_ = D2_OSR512_CMD;
                adc_wait_time_ = 2;
                break;
            }
            case 3: {
                d1_cmd_ = D1_OSR1024_CMD;
                d2_cmd_ = D2_OSR1024_CMD;
                adc_wait_time_ = 3;
                break;
            }
            case 4: {
                d1_cmd_ = D1_OSR2048_CMD;
                d2_cmd_ = D2_OSR2048_CMD;
                adc_wait_time_ = 5;
                break;
            }
            case 5: {
                d1_cmd_ = D1_OSR4096_CMD;
                d2_cmd_ = D2_OSR4096_CMD;
                adc_wait_time_ = 10;
                break;
            }
            case 6: {
                d1_cmd_ = D1_OSR8192_CMD;
                d2_cmd_ = D2_OSR8192_CMD;
                adc_wait_time_ = 19;
                break;
            }
            default: {
            }
        }

        state_.set(State::READ_C1);
    }

    void MS5837_30BA::run_task() {
        uint8_t retry_times = RETRY_TIMES;
        uint8_t cmd = 0;
        switch (state_.get()) {
            case State::INITIALIZING: {
                cmd = RESET_CMD;
                get_peripheral<peripheral::I2CPeripheral>()->send_by_dma(ADDR, &cmd, 1);
                if (xSemaphoreTake(i2c_dma_tx_sem_, pdMS_TO_TICKS(TX_TIMEOUT)) == pdTRUE) {
                    // tx successful
                    vTaskDelay(100);
                    state_.set(State::READ_C1);
                    return;
                }
                vTaskDelay(100);
                break;
            }
            case State::READ_C1: {
                c1_pressure_sensitivity_ = read_calibration_data(1, &retry_times);
                if (c1_pressure_sensitivity_ != 0) {
                    state_.set(State::READ_C2);
                    return;
                }
                break;
            }
            case State::READ_C2: {
                c2_pressure_offset_ = read_calibration_data(2, &retry_times);
                if (c2_pressure_offset_ != 0) {
                    state_.set(State::READ_C3);
                    return;
                }
                break;
            }
            case State::READ_C3: {
                c3_temperature_coefficient_of_pressure_sensitivity_ = read_calibration_data(3, &retry_times);
                if (c3_temperature_coefficient_of_pressure_sensitivity_ != 0) {
                    state_.set(State::READ_C4);
                    return;
                }
                break;
            }
            case State::READ_C4: {
                c4_temperature_coefficient_of_pressure_offset_ = read_calibration_data(4, &retry_times);
                if (c4_temperature_coefficient_of_pressure_offset_ != 0) {
                    state_.set(State::READ_C5);
                    return;
                }
                break;
            }
            case State::READ_C5: {
                c5_reference_temperature_ = read_calibration_data(4, &retry_times);
                if (c5_reference_temperature_ != 0) {
                    state_.set(State::READ_C6);
                    return;
                }
                break;
            }
            case State::READ_C6: {
                c6_temperature_coefficient_of_the_temperature_ = read_calibration_data(4, &retry_times);
                if (c6_temperature_coefficient_of_the_temperature_ != 0) {
                    state_.set(State::READ_D1);
                    return;
                }
                break;
            }
            case State::READ_D1: {
                cmd = d1_cmd_;
                if (start_adc(cmd, &retry_times)) {
                    // waiting adc
                    vTaskDelay(adc_wait_time_);
                    // read adc result
                    d1_digital_pressure_value_ = read_adc_data(&retry_times);
                    if (d1_digital_pressure_value_ != 0) {
                        state_.set(State::READ_D2);
                        return;
                    }
                }
                break;
            }
            case State::READ_D2: {
                cmd = d2_cmd_;
                if (start_adc(cmd, &retry_times)) {
                    vTaskDelay(adc_wait_time_);
                    d2_digital_temperature_value_ = read_adc_data(&retry_times);
                    if (d2_digital_temperature_value_ != 0) {
                        state_.set(State::CALCULATE);
                        return;
                    }
                }
                break;
            }
            case State::CALCULATE: {
                dt_ = static_cast<int32_t>(d2_digital_temperature_value_) -
                      static_cast<int32_t>(c5_reference_temperature_) *
                      static_cast<int32_t>(256);
                temp_ = static_cast<int32_t>(
                    static_cast<int64_t>(2000) +
                    static_cast<int64_t>(dt_) *
                    static_cast<int64_t>(c6_temperature_coefficient_of_the_temperature_) /
                    static_cast<int64_t>(8388608)
                );
                off_ = static_cast<int64_t>(c2_pressure_offset_) *
                       static_cast<int64_t>(65535) +
                       static_cast<int64_t>(c4_temperature_coefficient_of_pressure_offset_) *
                       static_cast<int64_t>(dt_) /
                       static_cast<int64_t>(128);
                sens_ = static_cast<int64_t>(c1_pressure_sensitivity_) *
                        static_cast<int64_t>(32768) +
                        static_cast<int64_t>(c3_temperature_coefficient_of_pressure_sensitivity_) *
                        static_cast<int64_t>(dt_) /
                        static_cast<int64_t>(256);
                p_ = static_cast<int32_t>(
                    (
                        static_cast<int64_t>(d1_digital_pressure_value_) *
                        sens_ /
                        static_cast<int64_t>(2097152) - off_
                    ) /
                    static_cast<int64_t>(8192)
                );

                if (temp_ / 100 < 20) {
                    ti_ = static_cast<int64_t>(3) *
                          static_cast<int64_t>(dt_) *
                          static_cast<int64_t>(dt_) /
                          8589934592;
                    offi_ = static_cast<int64_t>(3) *
                            (static_cast<int64_t>(temp_) - static_cast<int64_t>(2000)) *
                            (static_cast<int64_t>(temp_) - static_cast<int64_t>(2000)) /
                            static_cast<int64_t>(2);
                    sensi_ = static_cast<int64_t>(5) *
                             (static_cast<int64_t>(temp_) - static_cast<int64_t>(2000)) *
                             (static_cast<int64_t>(temp_) - static_cast<int64_t>(2000)) /
                             static_cast<int64_t>(8);
                    if (temp_ / 100 < -15) {
                        offi_ = offi_ +
                                static_cast<int64_t>(7) *
                                (static_cast<int64_t>(temp_) + static_cast<int64_t>(1500)) *
                                (static_cast<int64_t>(temp_) + static_cast<int64_t>(1500));
                        sensi_ = sensi_ +
                                 static_cast<int64_t>(4) *
                                 (static_cast<int64_t>(temp_) + static_cast<int64_t>(1500)) *
                                 (static_cast<int64_t>(temp_) + static_cast<int64_t>(1500));
                    }
                } else {
                    ti_ = static_cast<int64_t>(2) *
                          static_cast<int64_t>(dt_) *
                          static_cast<int64_t>(dt_) /
                          137438953472;
                    offi_ = (static_cast<int64_t>(temp_) - static_cast<int64_t>(2000)) *
                            (static_cast<int64_t>(temp_) - static_cast<int64_t>(2000)) /
                            static_cast<int64_t>(16);
                    sensi_ = 0;
                }

                off2_ = off_ - offi_;
                sens2_ = sens_ - sensi_;

                temp2_.set(static_cast<int32_t>(temp_ - ti_));
                p2_.set((
                            static_cast<int32_t>(d1_digital_pressure_value_) *
                            static_cast<int32_t>(sens2_) /
                            2097152 -
                            static_cast<int32_t>(off2_)
                        ) /
                        8192);

                state_.set(State::READ_D1);
                vTaskDelay(1);
                return;
            }
            default: {
            }
        }

        // if reach here means failed 3 times
        // mostly the slave is offline
        state_.set(State::INITIALIZING);
    }

    void MS5837_30BA::i2c_err() {
        get_peripheral<peripheral::I2CPeripheral>()->reset_tx_dma();
    }

    void MS5837_30BA::i2c_dma_tx_finished_callback() {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(i2c_dma_tx_sem_, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    void MS5837_30BA::i2c_recv() {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(i2c_dma_rx_sem_, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    void MS5837_30BA::write_to_master(buffer::Buffer *slave_to_master_buf) {
        slave_to_master_buf->write_int32(buffer::EndianType::LITTLE, temp2_.get());
        slave_to_master_buf->write_int32(buffer::EndianType::LITTLE, p2_.get());
    }
}
