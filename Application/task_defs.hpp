//
// Created by Hang XU on 04/06/2025.
//

#ifndef TASK_DEFS_H
#define TASK_DEFS_H

#include <memory>
#include <vector>
#include <atomic>
#include "c_task_warpper.h"
#include "peripheral_manager.hpp"
#include "pid.hpp"
#include "thread_safe_utils.hpp"
#include "crc16.hpp"

extern "C" {
#include "main.h"
}

namespace aim::ecat::task {
    using namespace utils::thread_safety;
    using namespace io;
    using namespace hardware;

    enum class TaskType : uint8_t {
        DBUS_RC = 1,
        LK_MOTOR = 2,
        HIPNUC_IMU_CAN = 3,
        DSHOT = 4,
        DJI_MOTOR = 5,
        ONBOARD_PWM = 6,
        EXTERNAL_PWM = 7,
        MS5876_30BA = 8,
        ADC = 9,
        CAN_PMU = 10,
        SBUS_RC = 11,
        DM_MOTOR = 12
    };

    class CustomRunnable {
    public:
        explicit CustomRunnable(const bool is_run_task_enabled) : is_run_task_enabled_(is_run_task_enabled) {
        }

        virtual ~CustomRunnable() = default;

        // unit: ms
        uint16_t period{};

        TaskType task_type{};

        ThreadSafeFlag running{true};

        bool is_run_task_enabled_{false};

        virtual void run_task() {
        }

        virtual void write_to_master(buffer::Buffer *slave_to_master_buf) {
            UNUSED(slave_to_master_buf);
        }

        virtual void read_from_master(buffer::Buffer *master_to_slave_buf) {
            UNUSED(master_to_slave_buf);
        }

        virtual void exit() {
            get_peripheral()->deinit();
        }

        void init_peripheral(const peripheral::Type type) {
            peripheral_ = peripheral::get_peripheral(type);
            get_peripheral()->init();
        }

        template<typename T = peripheral::Peripheral>
        [[nodiscard]] T *get_peripheral() const {
            return static_cast<T *>(peripheral_);
        }

    protected:
        peripheral::Peripheral *peripheral_{};
    };

    class CanRunnable : public CustomRunnable {
    public:
        explicit CanRunnable(const bool is_run_task_enabled) : CustomRunnable(is_run_task_enabled) {
        }

        ~CanRunnable() override = default;

        FDCAN_HandleTypeDef *can_inst_{};

        uint32_t can_id_type_{};

        virtual void can_recv(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data);

        void send_packet() const {
            HAL_FDCAN_AddMessageToTxFifoQ(can_inst_, &shared_tx_header_, shared_tx_buf_);
        }

    protected:
        FDCAN_TxHeaderTypeDef shared_tx_header_{};
        uint8_t shared_tx_buf_[8]{};
    };

    class UartRunnable : public CustomRunnable {
    public:
        explicit UartRunnable(const bool is_run_task_enabled) : CustomRunnable(is_run_task_enabled) {
        }

        ~UartRunnable() override = default;

        virtual void uart_recv(const uint16_t size) {
            UNUSED(size);
        }

        virtual void uart_err() {
        }

        virtual void uart_dma_tx_finished_callback() {
        }
    };

    class I2CRunnable : public CustomRunnable {
    public:
        explicit I2CRunnable(const bool is_run_task_enabled) : CustomRunnable(is_run_task_enabled) {
        }

        ~I2CRunnable() override = default;

        virtual void i2c_recv();

        virtual void i2c_err() {
        }

        virtual void i2c_dma_tx_finished_callback() {
        }
    };

    namespace dbus_rc {
        constexpr uint16_t DBUS_RC_CHANNAL_ERROR_VALUE = 1700;

        class DBUS_RC final : public UartRunnable {
        public:
            explicit DBUS_RC(buffer::Buffer *buffer);

            void write_to_master(buffer::Buffer *slave_to_master_buf) override;

            void uart_recv(uint16_t size) override;

            void uart_err() override;

        private:
            ThreadSafeTimestamp last_receive_time_{};
            ThreadSafeBuffer buf_{18};
        };
    }

    namespace ms5876 {
        constexpr uint8_t D1_OSR256_CMD = 0x40;
        constexpr uint8_t D1_OSR512_CMD = 0x42;
        constexpr uint8_t D1_OSR1024_CMD = 0x44;
        constexpr uint8_t D1_OSR2048_CMD = 0x46;
        constexpr uint8_t D1_OSR4096_CMD = 0x48;
        constexpr uint8_t D1_OSR8192_CMD = 0x4A;

        constexpr uint8_t D2_OSR256_CMD = 0x50;
        constexpr uint8_t D2_OSR512_CMD = 0x52;
        constexpr uint8_t D2_OSR1024_CMD = 0x54;
        constexpr uint8_t D2_OSR2048_CMD = 0x56;
        constexpr uint8_t D2_OSR4096_CMD = 0x58;
        constexpr uint8_t D2_OSR8192_CMD = 0x5A;

        constexpr uint8_t ADC_READ_CMD = 0x00;
        constexpr uint8_t PROM_READ_CMD_BEGIN = 0xA0;
        constexpr uint8_t RESET_CMD = 0x1E;
        constexpr uint8_t ADDR = 0x76 << 1;

        constexpr uint8_t RETRY_TIMES = 3;
        constexpr uint32_t TX_TIMEOUT = 50;

        enum class State : uint8_t {
            INITIALIZING = 0,

            READ_C1 = 1,
            READ_C2 = 2,
            READ_C3 = 3,
            READ_C4 = 4,
            READ_C5 = 5,
            READ_C6 = 6,

            READ_D1 = 7,
            READ_D2 = 8,

            CALCULATE = 9,
        };

        class MS5837_30BA final : public I2CRunnable {
        public:
            explicit MS5837_30BA(buffer::Buffer *buffer);

            void write_to_master(buffer::Buffer *slave_to_master_buf) override;

            void run_task() override;

            void i2c_err() override;

            void i2c_recv() override;

            void i2c_dma_tx_finished_callback() override;

        private:
            ThreadSafeValue<State> state_{State::INITIALIZING};
            SemaphoreHandle_t i2c_dma_tx_sem_{nullptr};
            SemaphoreHandle_t i2c_dma_rx_sem_{nullptr};

            uint8_t osr_id_{};
            uint8_t d1_cmd_{};
            uint8_t d2_cmd_{};
            uint32_t adc_wait_time_{};

            uint16_t c1_pressure_sensitivity_{};
            uint16_t c2_pressure_offset_{};
            uint16_t c3_temperature_coefficient_of_pressure_sensitivity_{};
            uint16_t c4_temperature_coefficient_of_pressure_offset_{};
            uint16_t c5_reference_temperature_{};
            uint16_t c6_temperature_coefficient_of_the_temperature_{};

            uint32_t d1_digital_pressure_value_{};
            uint32_t d2_digital_temperature_value_{};

            int32_t dt_{};
            int32_t temp_{};

            int64_t off_{};
            int64_t sens_{};
            int32_t p_{};

            int64_t ti_{};
            int64_t offi_{};
            int64_t sensi_{};
            int64_t off2_{};
            int64_t sens2_{};

            ThreadSafeValue<int32_t> temp2_{};
            ThreadSafeValue<int32_t> p2_{};

            uint16_t read_calibration_data(const int index, uint8_t *retry_times) const {
                uint8_t cmd = 0;
                uint16_t res = 0;
                cmd = PROM_READ_CMD_BEGIN + index * 2;
                get_peripheral<peripheral::I2CPeripheral>()->send_by_dma(ADDR, &cmd, 1);
                while ((*retry_times)--) {
                    if (xSemaphoreTake(i2c_dma_tx_sem_, pdMS_TO_TICKS(TX_TIMEOUT)) == pdTRUE) {
                        // tx finished, waiting for rx
                        get_peripheral<peripheral::I2CPeripheral>()->receive_by_dma(ADDR, 2);
                        if (xSemaphoreTake(i2c_dma_rx_sem_, pdMS_TO_TICKS(TX_TIMEOUT)) == pdTRUE) {
                            // rx finished, reading and switching to next state
                            res = get_peripheral<peripheral::UartPeripheral>()->recv_buf->read_uint16(
                                buffer::EndianType::BIG);
                            break;
                        }
                    }
                    get_peripheral<peripheral::I2CPeripheral>()->reset_tx_dma();
                }
                return res;
            }

            bool start_adc(const uint8_t cmd_used, uint8_t *retry_times) const {
                const uint8_t cmd = cmd_used;
                get_peripheral<peripheral::I2CPeripheral>()->send_by_dma(ADDR, &cmd, 1);
                while ((*retry_times)--) {
                    if (xSemaphoreTake(i2c_dma_tx_sem_, pdMS_TO_TICKS(TX_TIMEOUT)) == pdTRUE) {
                        return true;
                    }
                    vTaskDelay(adc_wait_time_);
                    get_peripheral<peripheral::I2CPeripheral>()->reset_tx_dma();
                }
                return false;
            }

            uint32_t read_adc_data(uint8_t *retry_times) const {
                // ReSharper disable once CppVariableCanBeMadeConstexpr
                const uint8_t cmd = ADC_READ_CMD;
                uint32_t res = 0;
                get_peripheral<peripheral::I2CPeripheral>()->send_by_dma(ADDR, &cmd, 1);
                while ((*retry_times)--) {
                    if (xSemaphoreTake(i2c_dma_tx_sem_, pdMS_TO_TICKS(TX_TIMEOUT)) == pdTRUE) {
                        // tx finished, waiting for rx
                        get_peripheral<peripheral::I2CPeripheral>()->receive_by_dma(ADDR, 2);
                        if (xSemaphoreTake(i2c_dma_rx_sem_, pdMS_TO_TICKS(TX_TIMEOUT)) == pdTRUE) {
                            // rx finished, reading and switching to next state
                            res = get_peripheral<peripheral::UartPeripheral>()->recv_buf->get_buf_pointer<uint8_t>()[0]
                                  << 16 |
                                  get_peripheral<peripheral::UartPeripheral>()->recv_buf->get_buf_pointer<uint8_t>()[1]
                                  << 8 |
                                  get_peripheral<peripheral::UartPeripheral>()->recv_buf->get_buf_pointer<uint8_t>()[2];
                            break;
                        }
                    }
                    vTaskDelay(adc_wait_time_);
                    get_peripheral<peripheral::I2CPeripheral>()->reset_tx_dma();
                }
                return res;
            }
        };
    }

    namespace dm_motor {
        enum class CtrlMode : uint32_t {
            MIT = 1,
            POSITION_WITH_SPEED_LIMIT = 2,
            SPEED = 3
        };

        enum class State {
            OFFLINE,
            SENDING_MODE_CHANGE,
            PENDING_MODE_CHANGE,
            MODE_CHANGED,
            MODE_CHANGE_BYPASSED,
        };

        struct MotorState {
            ThreadSafeValue<State> state{};
            ThreadSafeFlag is_motor_enabled{false};
            ThreadSafeTimestamp enter_timestamp{};
        };

        struct ControlCommand {
            ThreadSafeFlag is_enable{};
            ThreadSafeBuffer cmd{8};
        };

        // ms
        constexpr int MODE_CHANGE_BYPASS_TIMEOUT = 1000;

        class DM_MOTOR final : public CanRunnable {
        public:
            explicit DM_MOTOR(buffer::Buffer *buffer);

            void write_to_master(buffer::Buffer *slave_to_master_buf) override;

            void read_from_master(buffer::Buffer *master_to_slave_buf) override;

            void can_recv(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data) override;

            void run_task() override;

        private:
            // motor to ctrl
            uint32_t master_id_{};
            // ctrl to motor
            uint32_t can_id_{};

            ThreadSafeBuffer report_{8};
            ThreadSafeTimestamp last_receive_time{};

            ControlCommand cmd_{};
            CtrlMode mode_{};
            MotorState state_{};
            uint8_t mode_change_buf_[8]{};

            void change_state(const State new_state) {
                state_.state.set(new_state);
                state_.enter_timestamp.set_current();
            }

            [[nodiscard]] uint32_t get_state_enter_time() const {
                return state_.enter_timestamp.get();
            }

            [[nodiscard]] State get_current_state() const {
                return state_.state.get();
            }

            [[nodiscard]] uint32_t get_control_packet_id() const {
                switch (mode_) {
                    case CtrlMode::POSITION_WITH_SPEED_LIMIT: {
                        return can_id_ + 0x100;
                    }

                    case CtrlMode::SPEED: {
                        return can_id_ + 0x200;
                    }
                    case CtrlMode::MIT:
                    default: {
                        return can_id_;
                    }
                }
            }

            void generate_mode_change_packet() {
                memcpy(shared_tx_buf_, mode_change_buf_, 8);
            }

            void generate_disable_packet() {
                memset(shared_tx_buf_, 0xff, 7);
                shared_tx_buf_[7] = 0xfd;
            }

            void generate_enable_packet() {
                memset(shared_tx_buf_, 0xff, 7);
                shared_tx_buf_[7] = 0xfc;
            }

            [[nodiscard]] bool is_online() const {
                return HAL_GetTick() - last_receive_time.get() <= 50;
            }
        };
    }

    namespace adc {
        inline float lowpass_filter(const float prev, const float current, const float alpha) {
            return alpha * current + (1.0f - alpha) * prev;
        }

        constexpr float ADC_LF_ALPHA = 0.075;

        class ADC final : public CustomRunnable {
        public:
            explicit ADC(buffer::Buffer *buffer);

            void write_to_master(buffer::Buffer *slave_to_master_buf) override;

        private:
            ThreadSafeValue<float> parsed_adc_value_channel1{};
            ThreadSafeValue<float> parsed_adc_value_channel2{};
            float coefficient_[2]{};
        };
    }

    namespace dji_motor {
        struct MotorReport {
            ThreadSafeValue<uint16_t> ecd{};
            ThreadSafeValue<int16_t> rpm{};
            ThreadSafeValue<int16_t> current{};
            ThreadSafeValue<uint8_t> temperature{};
            ThreadSafeTimestamp last_receive_time{};
        };

        enum class CtrlMode : uint8_t {
            OPEN_LOOP_CURRENT = 0x01,
            SPEED = 0x02,
            SINGLE_ROUND_POSITION = 0x03
        };

        struct ControlCommand {
            ThreadSafeFlag is_enable{};
            ThreadSafeValue<int16_t> cmd{};
        };

        struct Motor {
            bool is_exist{false};

            MotorReport report{};
            CtrlMode mode{};
            ControlCommand command{};
            uint32_t report_packet_id{};
            uint8_t cmd_packet_idx{};

            algorithm::PID speed_pid{};
            algorithm::PID angle_pid{};

            [[nodiscard]] bool is_online() const {
                return HAL_GetTick() - report.last_receive_time.get() <= 50;
            }
        };

        inline int16_t calculate_err(const uint16_t current_angle, const uint16_t target_angle) {
            constexpr int total_positions = 8192;
            const int clockwise_difference = (target_angle - current_angle
                                              + total_positions) % total_positions;
            const int counterclockwise_difference = (current_angle - target_angle
                                                     + total_positions) % total_positions;
            if (clockwise_difference <= counterclockwise_difference) {
                return static_cast<int16_t>(clockwise_difference);
            }
            return static_cast<int16_t>(-counterclockwise_difference);
        }

        class DJI_MOTOR final : public CanRunnable {
        public:
            explicit DJI_MOTOR(buffer::Buffer *buffer);

            void write_to_master(buffer::Buffer *slave_to_master_buf) override;

            void read_from_master(buffer::Buffer *master_to_slave_buf) override;

            void can_recv(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data) override;

            void run_task() override;

        private:
            Motor motors_[4]{};
            int16_t cmds_[4]{};
        };
    }

    namespace hipnuc_imu {
        class HIPNUC_IMU_CAN final : public CanRunnable {
        public:
            explicit HIPNUC_IMU_CAN(buffer::Buffer *buffer);

            void write_to_master(buffer::Buffer *slave_to_master_buf) override;

            void can_recv(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data) override;

        private:
            ThreadSafeBuffer buf_{21};

            uint32_t packet1_id_{};
            uint32_t packet2_id_{};
            uint32_t packet3_id_{};
        };
    }

    namespace pmu_uavcan {
        constexpr int BUF_SIZE = 64;
        // ms
        constexpr int TID_TIMEOUT = 1000;
        constexpr uint32_t PACKET_ID = 0x1401557F;
        // ms
        constexpr int STATE_BROADCAST_PERIOD = 1000;

        struct RxState {
            uint8_t buffer[BUF_SIZE];
            uint16_t len;
            uint16_t crc;
            uint8_t initialized;
            uint8_t toggle;
            uint8_t transfer_id;
            uint32_t last_ts;
        };

        struct TailByte {
            uint8_t start;
            uint8_t end;
            uint8_t toggle;
            uint8_t tid;
        };

        class PMU_UAVCAN final : public CanRunnable {
        public:
            explicit PMU_UAVCAN(buffer::Buffer */* buffer */);

            void write_to_master(buffer::Buffer *slave_to_master_buf) override;

            void can_recv(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data) override;

            void run_task() override;

        private:
            ThreadSafeCounter uptime_{};
            ThreadSafeValue<uint8_t> transfer_id_{};
            ThreadSafeTimestamp last_receive_time_{};
            ThreadSafeBuffer recv_buf_{6};
            TailByte tail_{};
            RxState rx_state_{};

            void parse_tail_byte(const uint8_t tail) {
                tail_.start = tail >> 7 & 1;
                tail_.end = tail >> 6 & 1;
                tail_.toggle = tail >> 5 & 1;
                tail_.tid = tail & 0x1F;
            }
        };
    }

    namespace sbus_rc {
        class SBUS_RC final : public UartRunnable {
        public:
            explicit SBUS_RC(buffer::Buffer *buffer);

            void write_to_master(buffer::Buffer *slave_to_master_buf) override;

            void uart_recv(uint16_t size) override;

            void uart_err() override;

        private:
            ThreadSafeTimestamp last_receive_time_{};
            ThreadSafeBuffer buf_{23};
        };
    }

    namespace pwm {
        constexpr uint32_t TIM2_FREQ = 240000000;
        constexpr uint32_t TIM3_FREQ = 240000000;

        struct ControlCommand {
            uint16_t channel1{};
            uint16_t channel2{};
            uint16_t channel3{};
            uint16_t channel4{};
        };

        struct TIMSettingPair {
            uint16_t psc;
            uint16_t arr;
        };

        class PWM_ONBOARD final : public CustomRunnable {
        public:
            explicit PWM_ONBOARD(buffer::Buffer *buffer);

            void read_from_master(buffer::Buffer *master_to_slave_buf) override;

        private:
            TIM_HandleTypeDef *tim_inst_{nullptr};
            ControlCommand command_{};
            TIMSettingPair setting_pair_{};
            uint16_t expected_period_{};

            [[nodiscard]] uint32_t calculate_compare(const uint16_t expected_high_pulse) const {
                return static_cast<uint32_t>(lround(
                    static_cast<double>(expected_high_pulse) /
                    static_cast<double>(expected_period_) *
                    this->setting_pair_.arr
                ));
            }

            void send_signal() const {
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-volatile"
                __HAL_TIM_SET_COMPARE(tim_inst_, TIM_CHANNEL_1, command_.channel1);
                __HAL_TIM_SET_COMPARE(tim_inst_, TIM_CHANNEL_2, command_.channel2);
                __HAL_TIM_SET_COMPARE(tim_inst_, TIM_CHANNEL_3, command_.channel3);
                __HAL_TIM_SET_COMPARE(tim_inst_, TIM_CHANNEL_4, command_.channel4);
#pragma clang diagnostic pop
            }
        };

        struct ExternalServoBoardControlPacket {
            uint8_t header{0x01};
            uint16_t expected_period{};
            uint16_t servo_cmd[16]{};
            uint16_t checksum{};
        } __attribute__((packed));

        class PWM_EXTERNAL final : public UartRunnable {
        public:
            explicit PWM_EXTERNAL(buffer::Buffer *buffer);

            void read_from_master(buffer::Buffer *master_to_slave_buf) override;

            void uart_dma_tx_finished_callback() override;

            void uart_err() override;

            void run_task() override;

        private:
            uint8_t enabled_channel_count_{};
            uint16_t expected_period_{};
            ThreadSafeTimestamp last_send_time_{};
            ThreadSafeTimestamp last_send_finished_time_{};
            ThreadSafeTimestamp last_reset_time_{};
            ExternalServoBoardControlPacket control_packet_;

            void send_packet() {
                uint8_t cmd_buf[37] = {};
                memcpy(cmd_buf, &control_packet_, 37);
                algorithm::crc16::append_CRC16_check_sum(cmd_buf, 37);
                // if not busy, then return true, means data sent
                if (get_peripheral<peripheral::UartPeripheral>()->send_by_dma(cmd_buf, 37)) {
                    last_send_time_.set_current();
                }
            }
        };

        constexpr uint32_t DSHOT600_FREQ = 12000000;

        constexpr uint32_t DSHOT_DMA_BUFFER_SIZE = 18;
        constexpr uint8_t MOTOR_BIT_0 = 7;
        constexpr uint8_t MOTOR_BIT_1 = 14;
        constexpr uint32_t MOTOR_BITLENGTH = 20;
        constexpr uint32_t DSHOT_FRAME_SIZE = 16;

        inline uint16_t dshot_prepare_packet(const uint16_t value) {
            constexpr uint8_t dshot_telemetry = 0;
            uint16_t packet = value << 1 | dshot_telemetry;

            // compute checksum
            unsigned csum = 0;
            unsigned csum_data = packet;

            for (int i = 0; i < 3; i++) {
                // xor data by nibbles
                csum ^= csum_data;
                csum_data >>= 4;
            }

            csum &= 0xf;
            packet = packet << 4 | csum;

            return packet;
        }

        inline void dshot_prepare_dma_buffer(uint32_t *motor_dma_buffer, const uint16_t value) {
            constexpr uint8_t dshot_telemetry = 0;
            uint16_t packet = static_cast<uint16_t>(algorithm::limit_max_min(value, 2047, 0)) << 1 | dshot_telemetry;

            // compute checksum
            unsigned csum = 0;
            unsigned csum_data = packet;

            for (int i = 0; i < 3; i++) {
                // xor data by nibbles
                csum ^= csum_data;
                csum_data >>= 4;
            }

            csum &= 0xf;
            packet = packet << 4 | csum;

            for (int i = 0; i < 16; i++) {
                motor_dma_buffer[i] = packet & 0x8000 ? MOTOR_BIT_1 : MOTOR_BIT_0;
                packet <<= 1;
            }

            motor_dma_buffer[16] = 0;
            motor_dma_buffer[17] = 0;
        }

        class DSHOT600 final : public CustomRunnable {
        public:
            explicit DSHOT600(buffer::Buffer *buffer);

            void read_from_master(buffer::Buffer *master_to_slave_buf) override;

            void exit() override;

        private:
            TIM_HandleTypeDef *tim_inst_{nullptr};
            ControlCommand command_{};
            uint32_t *motor1_buffer{};
            uint32_t *motor2_buffer{};
            uint32_t *motor3_buffer{};
            uint32_t *motor4_buffer{};

            void init_dshot_dma(const uint32_t tim_freq) const {
                const uint16_t dshot_psc = lrintf(static_cast<float>(tim_freq) / DSHOT600_FREQ + 0.01f) - 1;
                __HAL_TIM_SET_PRESCALER(tim_inst_, dshot_psc);
                __HAL_TIM_SET_AUTORELOAD(tim_inst_, MOTOR_BITLENGTH);

                tim_inst_->hdma[TIM_DMA_ID_CC1]->XferCpltCallback = dshot_dma_tc_callback;
                tim_inst_->hdma[TIM_DMA_ID_CC2]->XferCpltCallback = dshot_dma_tc_callback;
                tim_inst_->hdma[TIM_DMA_ID_CC3]->XferCpltCallback = dshot_dma_tc_callback;
                tim_inst_->hdma[TIM_DMA_ID_CC4]->XferCpltCallback = dshot_dma_tc_callback;
            }

            void send_signal() const {
                send_dma_request(motor1_buffer, command_.channel1, &tim_inst_->Instance->CCR1, TIM_DMA_ID_CC1,
                                 TIM_DMA_CC1);
                send_dma_request(motor2_buffer, command_.channel2, &tim_inst_->Instance->CCR2, TIM_DMA_ID_CC2,
                                 TIM_DMA_CC2);
                send_dma_request(motor3_buffer, command_.channel3, &tim_inst_->Instance->CCR3, TIM_DMA_ID_CC3,
                                 TIM_DMA_CC3);
                send_dma_request(motor4_buffer, command_.channel4, &tim_inst_->Instance->CCR4, TIM_DMA_ID_CC4,
                                 TIM_DMA_CC4);
            }

            void send_dma_request(uint32_t *buffer, const uint16_t value, volatile uint32_t *ccr, const uint16_t dma_id,
                                  const uint32_t dma_cc) const {
                if (HAL_DMA_GetState(tim_inst_->hdma[dma_id]) == HAL_DMA_STATE_READY) {
                    dshot_prepare_dma_buffer(buffer, value);
                    HAL_DMA_Start_IT(tim_inst_->hdma[dma_id], reinterpret_cast<uint32_t>(buffer),
                                     reinterpret_cast<uint32_t>(ccr), DSHOT_DMA_BUFFER_SIZE);
                    __HAL_TIM_ENABLE_DMA(tim_inst_, dma_cc);
                }
            }
        };
    }

    namespace lk_motor {
        enum class State {
            DISABLED,
            QUERYING_STATE,
            ENABLED
        };

        struct MotorReport {
            ThreadSafeFlag is_motor_enabled{false};
            ThreadSafeValue<uint16_t> ecd{0};
            ThreadSafeValue<int16_t> speed{0};
            ThreadSafeValue<int16_t> current{0};
            ThreadSafeValue<uint8_t> temperature{0};
            ThreadSafeTimestamp last_receive_time{};
        };

        enum class CtrlMode : uint8_t {
            OPEN_LOOP_CURRENT = 0x01,
            TORQUE = 0x02,
            SPEED_WITH_TORQUE_LIMIT = 0x03,
            MULTI_ROUND_POSITION = 0x04,
            MULTI_ROUND_POSITION_WITH_SPEED_LIMIT = 0x05,
            SINGLE_ROUND_POSITION = 0x06,
            SINGLE_ROUND_POSITION_WITH_SPEED_LIMIT = 0x07,
        };

        struct ControlCommand {
            ThreadSafeFlag is_enable{};
            ThreadSafeFlag last_is_enable{};

            // for different control mode
            // valid for 0x06 0x07
            ThreadSafeValue<uint8_t> cmd_uint8{};
            // valid for 0x05 0x07
            ThreadSafeValue<uint16_t> cmd_uint16{};
            // valid for 0x01 0x02 0x03
            ThreadSafeValue<int16_t> cmd_int16{};
            // valid for 0x06 0x07
            ThreadSafeValue<uint32_t> cmd_uint32{};
            // valid for 0x03 0x04 0x05
            ThreadSafeValue<int32_t> cmd_int32{};
        };

        class LK_MOTOR final : public CanRunnable {
        public:
            explicit LK_MOTOR(buffer::Buffer *buffer);

            void write_to_master(buffer::Buffer *slave_to_master_buf) override;

            void read_from_master(buffer::Buffer *master_to_slave_buf) override;

            void can_recv(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data) override;

            void run_task() override;

        private:
            MotorReport report_{};
            CtrlMode mode_{};
            ThreadSafeValue<State> state_{State::DISABLED};
            ControlCommand command_{};
            uint32_t packet_id_{};

            [[nodiscard]] bool is_online() const {
                return HAL_GetTick() - report_.last_receive_time.get() <= 50;
            }

            void generate_disable_packet() {
                memset(shared_tx_buf_, 0, 8);
                shared_tx_buf_[0] = 0x80;
            }

            void generate_enable_packet() {
                memset(shared_tx_buf_, 0, 8);
                shared_tx_buf_[0] = 0x88;
            }

            void generate_state_check_packet() {
                memset(shared_tx_buf_, 0, 8);
                shared_tx_buf_[0] = 0x9A;
            }
        };
    }
}

#endif //TASK_DEFS_H