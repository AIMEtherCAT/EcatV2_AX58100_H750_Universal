//
// Created by Hang XU on 04/06/2025.
//

#ifndef TASK_DEFS_H
#define TASK_DEFS_H

#include <memory>
#include <vector>
#include <atomic>

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

        explicit CustomRunnable(const bool is_run_task_enabled) : is_run_task_enabled_(is_run_task_enabled) {}

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
        explicit CanRunnable(const bool is_run_task_enabled) : CustomRunnable(is_run_task_enabled) {}

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
        explicit UartRunnable(const bool is_run_task_enabled) : CustomRunnable(is_run_task_enabled) {}

        ~UartRunnable() override = default;

        virtual void uart_recv(uint16_t size);

        virtual void uart_err() {
        }

        virtual void uart_dma_tx_finished_callback() {
        }
    };

    // ReSharper disable once CppClassCanBeFinal
    class I2CRunnable : public CustomRunnable {
    public:
        explicit I2CRunnable(const bool is_run_task_enabled) : CustomRunnable(is_run_task_enabled) {}

        ~I2CRunnable() override = default;

        virtual void i2c_recv(uint16_t size);

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
        } ;

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
            ThreadSafeValue<uint16_t> channel1{};
            ThreadSafeValue<uint16_t> channel2{};
            ThreadSafeValue<uint16_t> channel3{};
            ThreadSafeValue<uint16_t> channel4{};
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
            ThreadSafeFlag is_pwm_started{false};
            uint16_t expected_period_{};

            [[nodiscard]] uint32_t calculate_compare(const uint16_t expected_high_pulse) const {
                return static_cast<uint32_t>(lround(
                    static_cast<double>(expected_high_pulse) /
                    static_cast<double>(expected_period_) *
                    this->setting_pair_.arr
                ));
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
            ThreadSafeTimestamp last_reset_time{};
            ExternalServoBoardControlPacket control_packet;

            void send_packet() {
                uint8_t cmd_buf[37] = {};
                memcpy(cmd_buf, &control_packet, 37);
                algorithm::crc16::append_CRC16_check_sum(cmd_buf, 37);
                // if not busy, then return true, means data sent
                if (get_peripheral<peripheral::UartPeripheral>()->send_by_dma(cmd_buf, 37)) {
                    last_send_time_.set_current();
                }
            }
        };

        class DSHOT600 final : public CustomRunnable {
        public:
            explicit DSHOT600(buffer::Buffer *buffer);

            void read_from_master(buffer::Buffer *master_to_slave_buf) override;

        private:
            TIM_HandleTypeDef *tim_inst_{nullptr};
            ControlCommand command_{};
            TIMSettingPair setting_pair_{};
            ThreadSafeFlag is_pwm_started{false};
            uint16_t expected_period_{};

            [[nodiscard]] uint32_t calculate_compare(const uint16_t expected_high_pulse) const {
                return static_cast<uint32_t>(lround(
                    static_cast<double>(expected_high_pulse) /
                    static_cast<double>(expected_period_) *
                    this->setting_pair_.arr
                ));
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
